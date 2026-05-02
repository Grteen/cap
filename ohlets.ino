// ============================================================================
// ohlets.ino — AI 컨베이어 분류 시스템 Arduino R4 WiFi 펌웨어
// belt: 인터럽트 기반 스텝 구동, 객체 위치: step 카운트 기반 추적
// ============================================================================

#include "FspTimer.h"
#include <math.h>
#include <string.h>

const int SORTER_STP_PIN = 10;
const int SORTER_DIR_PIN = 9;
const int SORTER_EN_PIN  = 8;

const int BELT_STP_PIN = 11;
const int BELT_DIR_PIN = 12;
const int BELT_EN_PIN  = 13;

const int IR_SENSOR_PIN = 7;

const long TIMER_PERIOD_US = 100;
const long SERIAL_BAUD_RATE = 115200;

const unsigned int STEP_PULSE_US = 6;
const unsigned int BELT_RAMP_STEP_HZ = 20;
const unsigned long BELT_RAMP_INTERVAL_MS = 10;
const unsigned int BELT_MIN_HZ = 100;
const unsigned int BELT_MAX_HZ = 10000;

const uint8_t PACKET_START_BYTE = 0xAA;
const uint8_t PACKET_END_BYTE   = 0xFF;

const uint8_t PACKET_TYPE_CLASS_RESULT = 0x01;  // object_id + class_flag
const uint8_t PACKET_TYPE_COEF         = 0x02;
const uint8_t PACKET_TYPE_CFG          = 0x03;

const uint8_t CLASS_RESULT_PAYLOAD_SIZE = 8;
const uint8_t COEF_PAYLOAD_SIZE         = 24;
const uint8_t CFG_PAYLOAD_SIZE          = 24;

const uint8_t CLASS_RESULT_PACKET_SIZE = 1 + 1 + CLASS_RESULT_PAYLOAD_SIZE + 1 + 1;
const uint8_t COEF_PACKET_SIZE         = 1 + 1 + COEF_PAYLOAD_SIZE + 1 + 1;
const uint8_t CFG_PACKET_SIZE          = 1 + 1 + CFG_PAYLOAD_SIZE + 1 + 1;

const uint8_t RX_BUFFER_MAX = 64;

volatile unsigned int runtimeBeltStepsPerSec = 2000;
volatile uint32_t runtimeIrToSorterDistanceSteps = 100000UL;
volatile float runtimeTFixed = 0.4f;
volatile float runtimeTHold = 0.8f;
volatile float runtimeTReturn = 0.4f;
volatile unsigned long runtimeIrDebounceMs = 500;

volatile unsigned int beltCurrentHz = BELT_MIN_HZ;
volatile unsigned int beltTargetHz = 2000;
unsigned long beltRampLastMs = 0;
volatile bool beltStoppedAtZeroHz = false;

volatile unsigned long beltStepCounter = 0;
volatile unsigned long beltStepAccumulator = 0;
volatile unsigned long beltStepThreshold = 1000000UL;

FspTimer isr_timer;

enum SorterState { SORTER_IDLE, SORTER_STRIKE, SORTER_HOLD, SORTER_RETURN };
volatile SorterState sorterState = SORTER_IDLE;

volatile float coeff_c0 = 0.0f;
volatile float coeff_c1 = 0.0f;
volatile float coeff_c2 = 0.0f;
volatile float coeff_c3 = 0.0f;
volatile float coeff_c4 = 0.0f;
volatile float coeff_c5 = 0.0f;

volatile unsigned long sorterStartTimeUs = 0;
volatile long sorterCurrentStep  = 0;
volatile float sorterDurationUs  = 0.0f;

struct TrackedObject {
  uint32_t id;
  uint32_t targetStep;
  int32_t classFlag;
  bool classified;
};

const uint8_t OBJECT_QUEUE_SIZE = 16;
volatile TrackedObject objectQueue[OBJECT_QUEUE_SIZE];
volatile uint8_t objHead = 0;
volatile uint8_t objTail = 0;
volatile uint8_t objCount = 0;
volatile uint32_t nextObjectId = 1;

uint8_t rxBuffer[RX_BUFFER_MAX];
int rxIndex = 0;
int expectedPacketSize = 0;

bool irLastState = false;
unsigned long irLastTriggerMs = 0;

uint8_t computeChecksum(uint8_t type, const uint8_t* payload, uint8_t payloadSize) {
  uint8_t cs = type;
  for (uint8_t i = 0; i < payloadSize; i++) cs ^= payload[i];
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

void emitStepPulse(int pin) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(pin, LOW);
}

void setBeltTargetHz(unsigned int hz) {
  if (hz > 0 && hz < BELT_MIN_HZ) hz = BELT_MIN_HZ;
  if (hz > BELT_MAX_HZ) hz = BELT_MAX_HZ;
  noInterrupts();
  beltTargetHz = hz;
  interrupts();
}

void updateBeltThresholdHz(unsigned int hz) {
  if (hz == 0) {
    noInterrupts();
    beltStoppedAtZeroHz = true;
    interrupts();
    return;
  }
  noInterrupts();
  beltStoppedAtZeroHz = false;
  beltStepThreshold = 1000000UL / (unsigned long)hz;
  interrupts();
}

void updateBeltRamp() {
  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - beltRampLastMs;
  if (elapsedMs < BELT_RAMP_INTERVAL_MS) return;

  unsigned long tickCount = elapsedMs / BELT_RAMP_INTERVAL_MS;
  beltRampLastMs += tickCount * BELT_RAMP_INTERVAL_MS;

  unsigned int currentHz;
  unsigned int targetHz;
  noInterrupts();
  currentHz = beltCurrentHz;
  targetHz = beltTargetHz;
  interrupts();

  if (currentHz == targetHz) return;

  unsigned long maxDelta = tickCount * BELT_RAMP_STEP_HZ;
  unsigned int nextHz = currentHz;

  if (currentHz < targetHz) {
    unsigned long v = (unsigned long)currentHz + maxDelta;
    nextHz = (v >= targetHz) ? targetHz : (unsigned int)v;
  } else {
    unsigned long gap = (unsigned long)currentHz - targetHz;
    nextHz = (maxDelta >= gap) ? targetHz : (unsigned int)(currentHz - maxDelta);
  }

  noInterrupts();
  beltCurrentHz = nextHz;
  interrupts();
  updateBeltThresholdHz(nextHz);
}

bool tryPopFrontObject(TrackedObject* outObj) {
  if (objCount == 0) return false;
  volatile TrackedObject* src = &objectQueue[objHead];
  outObj->id = src->id;
  outObj->targetStep = src->targetStep;
  outObj->classFlag = src->classFlag;
  outObj->classified = src->classified;
  objHead = (objHead + 1) % OBJECT_QUEUE_SIZE;
  objCount--;
  return true;
}

void startSorterStrike() {
  sorterStartTimeUs = micros();
  sorterDurationUs = runtimeTFixed * 1000000.0f;
  sorterCurrentStep = 0;
  digitalWrite(SORTER_DIR_PIN, HIGH);
  sorterState = SORTER_STRIKE;
}

void processDueObjects() {
  if (sorterState != SORTER_IDLE || objCount == 0) return;

  TrackedObject front;
  front.id = objectQueue[objHead].id;
  front.targetStep = objectQueue[objHead].targetStep;
  front.classFlag = objectQueue[objHead].classFlag;
  front.classified = objectQueue[objHead].classified;
  if (beltStepCounter < front.targetStep) return;

  TrackedObject popped;
  if (!tryPopFrontObject(&popped)) return;

  if (popped.classified && popped.classFlag == 1) {
    startSorterStrike();
  }
}

void applyApiDelayCompensation() {
  unsigned int desiredHz = runtimeBeltStepsPerSec;

  if (objCount > 0) {
    TrackedObject front;
    front.id = objectQueue[objHead].id;
    front.targetStep = objectQueue[objHead].targetStep;
    front.classFlag = objectQueue[objHead].classFlag;
    front.classified = objectQueue[objHead].classified;

    if (!front.classified) {
      unsigned long distanceTotal = runtimeIrToSorterDistanceSteps;
      if (distanceTotal > 0) {
        long originStep = (long)front.targetStep - (long)distanceTotal;
        long done = (long)beltStepCounter - originStep;
        if (done < 0) done = 0;
        if ((unsigned long)done > distanceTotal) done = (long)distanceTotal;

        float progress = (float)done / (float)distanceTotal;
        if (progress >= 0.8f) {
          desiredHz = 0;
        } else if (progress >= 0.5f) {
          desiredHz = (unsigned int)roundf((float)runtimeBeltStepsPerSec * 0.5f);
          if (desiredHz > 0 && desiredHz < BELT_MIN_HZ) desiredHz = BELT_MIN_HZ;
        }
      }
    }
  }

  setBeltTargetHz(desiredHz);
}

void timerISR(timer_callback_args_t __attribute((unused)) *args) {
  if (!beltStoppedAtZeroHz) {
    beltStepAccumulator += TIMER_PERIOD_US;
    if (beltStepAccumulator >= beltStepThreshold) {
      beltStepAccumulator -= beltStepThreshold;
      emitStepPulse(BELT_STP_PIN);
      beltStepCounter++;
    }
  }

  processDueObjects();

  unsigned long nowUs = micros();
  switch (sorterState) {
    case SORTER_IDLE:
      break;

    case SORTER_STRIKE: {
      float elapsedUs = (float)(nowUs - sorterStartTimeUs);
      float u = (elapsedUs >= sorterDurationUs) ? 1.0f : (elapsedUs / sorterDurationUs);
      long targetNow = (long)roundf(evaluatePolynomial(u));
      if (targetNow > sorterCurrentStep) {
        emitStepPulse(SORTER_STP_PIN);
        sorterCurrentStep++;
      }
      if (elapsedUs >= sorterDurationUs && sorterCurrentStep >= targetNow) {
        sorterState = SORTER_HOLD;
        sorterStartTimeUs = nowUs;
        sorterDurationUs = runtimeTHold * 1000000.0f;
      }
      break;
    }

    case SORTER_HOLD:
      if ((float)(nowUs - sorterStartTimeUs) >= sorterDurationUs) {
        sorterState = SORTER_RETURN;
        sorterStartTimeUs = nowUs;
        sorterDurationUs = runtimeTReturn * 1000000.0f;
        digitalWrite(SORTER_DIR_PIN, LOW);
      }
      break;

    case SORTER_RETURN: {
      float elapsedUs = (float)(nowUs - sorterStartTimeUs);
      long targetNow = 0;
      if (elapsedUs < sorterDurationUs) {
        float u = 1.0f - (elapsedUs / sorterDurationUs);
        targetNow = (long)roundf(evaluatePolynomial(u));
        if (targetNow < 0) targetNow = 0;
      }
      if (sorterCurrentStep > targetNow) {
        emitStepPulse(SORTER_STP_PIN);
        sorterCurrentStep--;
      }
      if (elapsedUs >= sorterDurationUs && sorterCurrentStep <= targetNow) {
        sorterCurrentStep = targetNow;
        sorterState = SORTER_IDLE;
      }
      break;
    }
  }}

void handleClassResultPayload(const uint8_t* payload) {
  uint32_t objectId = 0;
  int32_t classifyFlag = 0;
  memcpy(&objectId, payload, 4);
  memcpy(&classifyFlag, payload + 4, 4);

  noInterrupts();
  for (uint8_t i = 0; i < objCount; i++) {
    uint8_t idx = (objHead + i) % OBJECT_QUEUE_SIZE;
    if (objectQueue[idx].id == objectId) {
      objectQueue[idx].classFlag = classifyFlag;
      objectQueue[idx].classified = true;
      break;
    }
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
  coeff_c0 = c0; coeff_c1 = c1; coeff_c2 = c2;
  coeff_c3 = c3; coeff_c4 = c4; coeff_c5 = c5;
  interrupts();
  Serial.println("COEF_OK");
}

void handleCfgPayload(const uint8_t* payload) {
  float beltHzF, tFixed, tHold, tReturn;
  uint32_t debounceMs;
  uint32_t irToSorterDistanceSteps;
  memcpy(&beltHzF, payload + 0, 4);
  memcpy(&tFixed, payload + 4, 4);
  memcpy(&tHold, payload + 8, 4);
  memcpy(&tReturn, payload + 12, 4);
  memcpy(&debounceMs, payload + 16, 4);
  memcpy(&irToSorterDistanceSteps, payload + 20, 4);

  if (!isFloatValid(beltHzF) || !isFloatValid(tFixed) || !isFloatValid(tHold) || !isFloatValid(tReturn)) {
    Serial.println("CFG_ERR");
    return;
  }

  unsigned int beltHz = (unsigned int)roundf(beltHzF);
  if (beltHz < BELT_MIN_HZ || beltHz > BELT_MAX_HZ ||
      tFixed < 0.1f || tFixed > 5.0f ||
      tHold < 0.1f || tHold > 5.0f ||
      tReturn < 0.1f || tReturn > 5.0f ||
      debounceMs < 100 || debounceMs > 5000 ||
      irToSorterDistanceSteps < 1000UL || irToSorterDistanceSteps > 5000000UL) {
    Serial.println("CFG_ERR");
    return;
  }

  noInterrupts();
  bool busy = (sorterState != SORTER_IDLE);
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
  runtimeIrToSorterDistanceSteps = irToSorterDistanceSteps;
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

  if (type == PACKET_TYPE_CLASS_RESULT && payloadSize == CLASS_RESULT_PAYLOAD_SIZE) {
    handleClassResultPayload(payload);
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
    if (inByte == PACKET_START_BYTE) rxBuffer[rxIndex++] = inByte;
    return;
  }

  if (rxIndex == 1) {
    rxBuffer[rxIndex++] = inByte;
    uint8_t type = inByte;
    if (type == PACKET_TYPE_CLASS_RESULT) expectedPacketSize = CLASS_RESULT_PACKET_SIZE;
    else if (type == PACKET_TYPE_COEF) expectedPacketSize = COEF_PACKET_SIZE;
    else if (type == PACKET_TYPE_CFG) expectedPacketSize = CFG_PACKET_SIZE;
    else resetRxState();
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

void enqueueObjectFromIr() {
  if (objCount >= OBJECT_QUEUE_SIZE) {
    TrackedObject dropped;
    tryPopFrontObject(&dropped);
  }

  TrackedObject obj;
  obj.id = nextObjectId++;
  obj.targetStep = beltStepCounter + runtimeIrToSorterDistanceSteps;
  obj.classFlag = 0;
  obj.classified = false;

  objectQueue[objTail].id = obj.id;
  objectQueue[objTail].targetStep = obj.targetStep;
  objectQueue[objTail].classFlag = obj.classFlag;
  objectQueue[objTail].classified = obj.classified;
  objTail = (objTail + 1) % OBJECT_QUEUE_SIZE;
  objCount++;

  Serial.print("IR_DETECTED:");
  Serial.println(obj.id);
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
  updateBeltThresholdHz(beltCurrentHz);

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
  applyApiDelayCompensation();
  updateBeltRamp();

  bool irCurrentState = (digitalRead(IR_SENSOR_PIN) == HIGH);
  unsigned long nowMs = millis();

  if (irCurrentState && !irLastState) {
    if ((nowMs - irLastTriggerMs) >= runtimeIrDebounceMs) {
      noInterrupts();
      enqueueObjectFromIr();
      interrupts();
      irLastTriggerMs = nowMs;
    }
  }
  irLastState = irCurrentState;

  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    parseIncomingByte(inByte);
  }
}
