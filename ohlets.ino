// ============================================================================
// ohlets.ino — AI 컨베이어 분류 시스템 Arduino R4 WiFi 펌웨어
// Python main.py / dashboard.py 런타임 설정 동기화 버전
// ============================================================================

#include "FspTimer.h"
#include <math.h>

// ==================== 고정 하드웨어 상수 ====================
const int SORTER_STP_PIN = 10;
const int SORTER_DIR_PIN = 9;
const int SORTER_EN_PIN  = 8;

const int BELT_STP_PIN = 11;
const int BELT_DIR_PIN = 12;
const int BELT_EN_PIN  = 13;

const int IR_SENSOR_PIN = 7;

const long TIMER_PERIOD_US = 100;
const long SERIAL_BAUD_RATE = 115200;

const unsigned int SORTER_STEP_PULSE_US = 8;
const unsigned int BELT_RAMP_STEP_HZ = 20;
const unsigned long BELT_RAMP_INTERVAL_MS = 10;
const unsigned int BELT_MIN_HZ = 100;
const unsigned int BELT_MAX_HZ = 10000;

// ==================== 프로토콜 상수 ====================
const uint8_t PACKET_START_BYTE = 0xAA;
const uint8_t PACKET_END_BYTE   = 0xFF;

const uint8_t PACKET_TYPE_EVENT = 0x01;  // flag + eta
const uint8_t PACKET_TYPE_COEF  = 0x02;  // c0..c5
const uint8_t PACKET_TYPE_CFG   = 0x03;  // belt,t_fixed,t_hold,t_return,debounce

const uint8_t EVENT_PAYLOAD_SIZE = 8;
const uint8_t COEF_PAYLOAD_SIZE  = 24;
const uint8_t CFG_PAYLOAD_SIZE   = 20;

const uint8_t EVENT_PACKET_SIZE = 1 + 1 + EVENT_PAYLOAD_SIZE + 1 + 1;
const uint8_t COEF_PACKET_SIZE  = 1 + 1 + COEF_PAYLOAD_SIZE + 1 + 1;
const uint8_t CFG_PACKET_SIZE   = 1 + 1 + CFG_PAYLOAD_SIZE + 1 + 1;

const uint8_t RX_BUFFER_MAX = 64;

// ==================== 런타임 설정값 (Python에서 변경 가능) ====================
volatile unsigned int runtimeBeltStepsPerSec = 2000;
volatile float runtimeTFixed = 0.4f;
volatile float runtimeTHold = 0.8f;
volatile float runtimeTReturn = 0.4f;
volatile unsigned long runtimeIrDebounceMs = 500;

unsigned int beltCurrentHz = BELT_MIN_HZ;
unsigned int beltTargetHz = 2000;
unsigned long beltRampLastMs = 0;

// ==================== 전역 상태 ====================
FspTimer isr_timer;

enum SorterState {
  SORTER_IDLE,
  SORTER_WAIT,
  SORTER_STRIKE,
  SORTER_HOLD,
  SORTER_RETURN
};

volatile SorterState sorterState = SORTER_IDLE;

volatile float coeff_c0 = 0.0f;
volatile float coeff_c1 = 0.0f;
volatile float coeff_c2 = 0.0f;
volatile float coeff_c3 = 0.0f;
volatile float coeff_c4 = 0.0f;
volatile float coeff_c5 = 0.0f;

volatile unsigned long sorterStartTimeUs = 0;
volatile unsigned long sorterWaitStartUs = 0;
volatile unsigned long sorterWaitDurUs   = 0;
volatile long sorterCurrentStep  = 0;
volatile float sorterDurationUs  = 0.0f;

// 수신 패킷 파서 상태
uint8_t rxBuffer[RX_BUFFER_MAX];
int rxIndex = 0;
int expectedPacketSize = 0;

bool irLastState = false;
unsigned long irLastTriggerMs = 0;

// ==================== 분류 커맨드 큐 ====================
struct SorterCommand {
  int32_t flag;
  float eta;
  unsigned long receivedAtUs;
};

const int CMD_QUEUE_SIZE = 5;
SorterCommand cmdQueue[CMD_QUEUE_SIZE];
volatile int cmdQHead = 0;
volatile int cmdQTail = 0;
volatile int cmdQCount = 0;

uint8_t computeChecksum(uint8_t type, const uint8_t* payload, uint8_t payloadSize) {
  uint8_t cs = type;
  for (uint8_t i = 0; i < payloadSize; i++) {
    cs ^= payload[i];
  }
  return cs;
}

bool isFloatValid(float v) {
  return !(isnan(v) || isinf(v));
}

float evaluatePolynomial(float u) {
  float s = coeff_c5;
  s = s * u + coeff_c4;
  s = s * u + coeff_c3;
  s = s * u + coeff_c2;
  s = s * u + coeff_c1;
  s = s * u + coeff_c0;
  return s;
}

void emitOneSorterStep() {
  digitalWrite(SORTER_STP_PIN, HIGH);
  delayMicroseconds(SORTER_STEP_PULSE_US);
  digitalWrite(SORTER_STP_PIN, LOW);
}

void setBeltTargetHz(unsigned int hz) {
  if (hz < BELT_MIN_HZ) hz = BELT_MIN_HZ;
  if (hz > BELT_MAX_HZ) hz = BELT_MAX_HZ;
  beltTargetHz = hz;
}

void updateBeltRamp() {
  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - beltRampLastMs;
  if (elapsedMs < BELT_RAMP_INTERVAL_MS) {
    return;
  }

  unsigned long tickCount = elapsedMs / BELT_RAMP_INTERVAL_MS;
  beltRampLastMs += tickCount * BELT_RAMP_INTERVAL_MS;

  if (beltCurrentHz == beltTargetHz) {
    return;
  }

  unsigned long maxDelta = tickCount * BELT_RAMP_STEP_HZ;

  if (beltCurrentHz < beltTargetHz) {
    unsigned long nextHz = (unsigned long)beltCurrentHz + maxDelta;
    beltCurrentHz = (nextHz >= beltTargetHz) ? beltTargetHz : (unsigned int)nextHz;
  } else {
    unsigned long currentHz = beltCurrentHz;
    beltCurrentHz = (maxDelta >= currentHz - beltTargetHz)
      ? beltTargetHz
      : (unsigned int)(currentHz - maxDelta);
  }

  tone(BELT_STP_PIN, beltCurrentHz);
}

void startNextFromQueue() {
  while (cmdQCount > 0) {
    SorterCommand cmd = cmdQueue[cmdQHead];
    cmdQHead = (cmdQHead + 1) % CMD_QUEUE_SIZE;
    cmdQCount--;

    if (cmd.flag == 0) continue;

    float elapsedSec  = (float)(micros() - cmd.receivedAtUs) / 1000000.0f;
    float adjustedEta = cmd.eta - elapsedSec;
    if (adjustedEta <= 0.0f) continue;

    float tWait = adjustedEta - runtimeTFixed;
    if (tWait > 0.0f) {
      sorterWaitStartUs = micros();
      sorterWaitDurUs   = (unsigned long)(tWait * 1000000.0f);
      sorterState = SORTER_WAIT;
    } else {
      sorterStartTimeUs = micros();
      sorterDurationUs = runtimeTFixed * 1000000.0f;
      sorterCurrentStep = 0;
      digitalWrite(SORTER_DIR_PIN, HIGH);
      sorterState = SORTER_STRIKE;
    }
    return;
  }
}

void timerISR(timer_callback_args_t __attribute((unused)) *args) {
  unsigned long nowUs = micros();

  switch (sorterState) {
    case SORTER_IDLE:
      break;

    case SORTER_WAIT: {
      if ((nowUs - sorterWaitStartUs) >= sorterWaitDurUs) {
        sorterState = SORTER_STRIKE;
        sorterStartTimeUs = nowUs;
        sorterDurationUs = runtimeTFixed * 1000000.0f;
        sorterCurrentStep = 0;
        digitalWrite(SORTER_DIR_PIN, HIGH);
      }
      break;
    }

    case SORTER_STRIKE: {
      float elapsedUs = (float)(nowUs - sorterStartTimeUs);
      float u = (elapsedUs >= sorterDurationUs) ? 1.0f : (elapsedUs / sorterDurationUs);
      long targetNow = (long)roundf(evaluatePolynomial(u));
      if (targetNow > sorterCurrentStep) {
        emitOneSorterStep();
        sorterCurrentStep++;
      }

      if (elapsedUs >= sorterDurationUs && sorterCurrentStep >= targetNow) {
        sorterState = SORTER_HOLD;
        sorterStartTimeUs = nowUs;
        sorterDurationUs = runtimeTHold * 1000000.0f;
      }
      break;
    }

    case SORTER_HOLD: {
      float elapsedUs = (float)(nowUs - sorterStartTimeUs);
      if (elapsedUs >= sorterDurationUs) {
        sorterState = SORTER_RETURN;
        sorterStartTimeUs = nowUs;
        sorterDurationUs = runtimeTReturn * 1000000.0f;
        digitalWrite(SORTER_DIR_PIN, LOW);
      }
      break;
    }

    case SORTER_RETURN: {
      float elapsedUs = (float)(nowUs - sorterStartTimeUs);
      long targetNow = 0;
      if (elapsedUs < sorterDurationUs) {
        float u = 1.0f - (elapsedUs / sorterDurationUs);
        targetNow = (long)roundf(evaluatePolynomial(u));
        if (targetNow < 0) {
          targetNow = 0;
        }
      }

      if (sorterCurrentStep > targetNow) {
        emitOneSorterStep();
        sorterCurrentStep--;
      }

      if (elapsedUs >= sorterDurationUs && sorterCurrentStep <= targetNow) {
        sorterCurrentStep = targetNow;
        sorterState = SORTER_IDLE;
        startNextFromQueue();
      }
      break;
    }
  }
}

void handleEventPayload(const uint8_t* payload) {
  int32_t classifyFlag = 0;
  float eta = 0.0f;
  memcpy(&classifyFlag, payload, 4);
  memcpy(&eta, payload + 4, 4);

  if (classifyFlag == 0) {
    return;
  }

  noInterrupts();

  if (sorterState != SORTER_IDLE) {
    if (cmdQCount < CMD_QUEUE_SIZE) {
      SorterCommand cmd;
      cmd.flag = classifyFlag;
      cmd.eta = eta;
      cmd.receivedAtUs = micros();
      cmdQueue[cmdQTail] = cmd;
      cmdQTail = (cmdQTail + 1) % CMD_QUEUE_SIZE;
      cmdQCount++;
    }
    interrupts();
    return;
  }

  float tWait = eta - runtimeTFixed;
  if (tWait > 0.0f) {
    sorterWaitStartUs = micros();
    sorterWaitDurUs = (unsigned long)(tWait * 1000000.0f);
    sorterState = SORTER_WAIT;
  } else {
    sorterStartTimeUs = micros();
    sorterDurationUs = runtimeTFixed * 1000000.0f;
    sorterCurrentStep = 0;
    digitalWrite(SORTER_DIR_PIN, HIGH);
    sorterState = SORTER_STRIKE;
  }

  interrupts();
}

void handleCoeffPayload(const uint8_t* payload) {
  float c0, c1, c2, c3, c4, c5;
  memcpy(&c0, payload + 0, 4);
  memcpy(&c1, payload + 4, 4);
  memcpy(&c2, payload + 8, 4);
  memcpy(&c3, payload + 12, 4);
  memcpy(&c4, payload + 16, 4);
  memcpy(&c5, payload + 20, 4);

  if (!isFloatValid(c0) || !isFloatValid(c1) || !isFloatValid(c2) ||
      !isFloatValid(c3) || !isFloatValid(c4) || !isFloatValid(c5)) {
    Serial.println("COEF_ERR");
    return;
  }

  noInterrupts();
  coeff_c0 = c0;
  coeff_c1 = c1;
  coeff_c2 = c2;
  coeff_c3 = c3;
  coeff_c4 = c4;
  coeff_c5 = c5;
  interrupts();

  Serial.println("COEF_OK");
}

void handleCfgPayload(const uint8_t* payload) {
  float beltHzF, tFixed, tHold, tReturn;
  uint32_t debounceMs;

  memcpy(&beltHzF, payload + 0, 4);
  memcpy(&tFixed,  payload + 4, 4);
  memcpy(&tHold,   payload + 8, 4);
  memcpy(&tReturn, payload + 12, 4);
  memcpy(&debounceMs, payload + 16, 4);

  if (!isFloatValid(beltHzF) || !isFloatValid(tFixed) || !isFloatValid(tHold) || !isFloatValid(tReturn)) {
    Serial.println("CFG_ERR");
    return;
  }

  unsigned int beltHz = (unsigned int)roundf(beltHzF);

  if (beltHz < BELT_MIN_HZ || beltHz > BELT_MAX_HZ ||
      tFixed < 0.1f || tFixed > 5.0f ||
      tHold < 0.1f || tHold > 5.0f ||
      tReturn < 0.1f || tReturn > 5.0f ||
      debounceMs < 100 || debounceMs > 5000) {
    Serial.println("CFG_ERR");
    return;
  }

  noInterrupts();
  bool busy = (sorterState != SORTER_IDLE) || (cmdQCount > 0);
  interrupts();

  if (busy) {
    Serial.println("CFG_BUSY");
    return;
  }

  noInterrupts();
  runtimeBeltStepsPerSec = beltHz;
  runtimeTFixed = tFixed;
  runtimeTHold = tHold;
  runtimeTReturn = tReturn;
  runtimeIrDebounceMs = debounceMs;
  interrupts();

  setBeltTargetHz(beltHz);

  Serial.println("CFG_OK");
}

void processPacket(const uint8_t* packet, int packetSize) {
  if (packetSize < 5) return;
  if (packet[0] != PACKET_START_BYTE || packet[packetSize - 1] != PACKET_END_BYTE) return;

  uint8_t type = packet[1];
  int payloadSize = packetSize - 4;
  const uint8_t* payload = packet + 2;
  uint8_t receivedChecksum = packet[packetSize - 2];

  if (computeChecksum(type, payload, (uint8_t)payloadSize) != receivedChecksum) {
    if (type == PACKET_TYPE_CFG) Serial.println("CFG_ERR");
    if (type == PACKET_TYPE_COEF) Serial.println("COEF_ERR");
    return;
  }

  if (type == PACKET_TYPE_EVENT && payloadSize == EVENT_PAYLOAD_SIZE) {
    handleEventPayload(payload);
  } else if (type == PACKET_TYPE_COEF && payloadSize == COEF_PAYLOAD_SIZE) {
    handleCoeffPayload(payload);
  } else if (type == PACKET_TYPE_CFG && payloadSize == CFG_PAYLOAD_SIZE) {
    handleCfgPayload(payload);
  } else {
    if (type == PACKET_TYPE_CFG) Serial.println("CFG_ERR");
    if (type == PACKET_TYPE_COEF) Serial.println("COEF_ERR");
  }
}

void resetRxState() {
  rxIndex = 0;
  expectedPacketSize = 0;
}

void parseIncomingByte(uint8_t inByte) {
  if (rxIndex == 0) {
    if (inByte == PACKET_START_BYTE) {
      rxBuffer[rxIndex++] = inByte;
    }
    return;
  }

  if (rxIndex == 1) {
    rxBuffer[rxIndex++] = inByte;
    uint8_t type = inByte;
    if (type == PACKET_TYPE_EVENT) {
      expectedPacketSize = EVENT_PACKET_SIZE;
    } else if (type == PACKET_TYPE_COEF) {
      expectedPacketSize = COEF_PACKET_SIZE;
    } else if (type == PACKET_TYPE_CFG) {
      expectedPacketSize = CFG_PACKET_SIZE;
    } else {
      resetRxState();
    }
    return;
  }

  if (rxIndex >= RX_BUFFER_MAX) {
    resetRxState();
    return;
  }

  rxBuffer[rxIndex++] = inByte;
  if (expectedPacketSize > 0 && rxIndex >= expectedPacketSize) {
    processPacket(rxBuffer, expectedPacketSize);
    resetRxState();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(SORTER_STP_PIN, OUTPUT);
  pinMode(SORTER_DIR_PIN, OUTPUT);
  pinMode(SORTER_EN_PIN, OUTPUT);

  pinMode(BELT_STP_PIN, OUTPUT);
  pinMode(BELT_DIR_PIN, OUTPUT);
  pinMode(BELT_EN_PIN, OUTPUT);

  pinMode(IR_SENSOR_PIN, INPUT);

  digitalWrite(SORTER_EN_PIN, LOW);
  digitalWrite(BELT_EN_PIN, LOW);

  digitalWrite(BELT_DIR_PIN, HIGH);
  beltCurrentHz = BELT_MIN_HZ;
  beltTargetHz = runtimeBeltStepsPerSec;
  beltRampLastMs = millis();
  tone(BELT_STP_PIN, beltCurrentHz);

  sorterState = SORTER_IDLE;

  uint8_t timerType = GPT_TIMER;
  int8_t timerChannel = FspTimer::get_available_timer(timerType);

  if (timerChannel >= 0) {
    float freqHz = 1000000.0f / (float)TIMER_PERIOD_US;
    isr_timer.begin(TIMER_MODE_PERIODIC, timerType, (uint8_t)timerChannel, freqHz, 0.0f, timerISR);
    isr_timer.setup_overflow_irq();
    isr_timer.open();
    isr_timer.start();
  }
}

void loop() {
  updateBeltRamp();

  bool irCurrentState = (digitalRead(IR_SENSOR_PIN) == HIGH);
  unsigned long nowMs = millis();

  if (irCurrentState && !irLastState) {
    if ((nowMs - irLastTriggerMs) >= runtimeIrDebounceMs) {
      Serial.println("IR_DETECTED");
      irLastTriggerMs = nowMs;
    }
  }
  irLastState = irCurrentState;

  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    parseIncomingByte(inByte);
  }
}
