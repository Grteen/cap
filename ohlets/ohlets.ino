// ============================================================================
// ohlets.ino — AI conveyor sorter firmware (step-based event tracking)
// ============================================================================

#include "FspTimer.h"
#include <math.h>

// ==================== Hardware constants ====================
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
const unsigned int BELT_STEP_PULSE_US = 4;
const unsigned int BELT_RAMP_STEP_HZ = 20;
const unsigned long BELT_RAMP_INTERVAL_MS = 10;
const unsigned int BELT_MIN_HZ = 100;
const unsigned int BELT_MAX_HZ = 10000;

// Fixed in firmware by request.
const int32_t IR_TO_SORTER_DISTANCE_STEPS = 100000;

// ==================== Protocol constants ====================
const uint8_t PACKET_START_BYTE = 0xAA;
const uint8_t PACKET_END_BYTE   = 0xFF;

const uint8_t PACKET_TYPE_EVENT = 0x01;  // event_id + classify_flag
const uint8_t PACKET_TYPE_COEF  = 0x02;  // c0..c5
const uint8_t PACKET_TYPE_CFG   = 0x03;  // belt,t_fixed,t_hold,t_return,debounce
const uint8_t PACKET_TYPE_TEST  = 0x04;  // no payload

const uint8_t EVENT_PAYLOAD_SIZE = 8;
const uint8_t COEF_PAYLOAD_SIZE  = 24;
const uint8_t CFG_PAYLOAD_SIZE   = 20;
const uint8_t TEST_PAYLOAD_SIZE  = 0;

const uint8_t EVENT_PACKET_SIZE = 1 + 1 + EVENT_PAYLOAD_SIZE + 1 + 1;
const uint8_t COEF_PACKET_SIZE  = 1 + 1 + COEF_PAYLOAD_SIZE + 1 + 1;
const uint8_t CFG_PACKET_SIZE   = 1 + 1 + CFG_PAYLOAD_SIZE + 1 + 1;
const uint8_t TEST_PACKET_SIZE  = 1 + 1 + TEST_PAYLOAD_SIZE + 1 + 1;

const uint8_t RX_BUFFER_MAX = 64;

// ==================== Runtime settings ====================
volatile unsigned int runtimeBeltStepsPerSec = 2000;
volatile float runtimeTFixed = 0.4f;
volatile float runtimeTHold = 0.8f;
volatile float runtimeTReturn = 0.4f;
volatile unsigned long runtimeIrDebounceMs = 500;

volatile unsigned int beltCurrentHz = BELT_MIN_HZ;
volatile unsigned int beltTargetHz = 2000;
unsigned long beltRampLastMs = 0;

// ==================== Global state ====================
FspTimer isr_timer;

enum SorterState {
  SORTER_IDLE,
  SORTER_WAIT_STEP,
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
volatile long sorterCurrentStep = 0;
volatile float sorterDurationUs = 0.0f;
volatile int64_t sorterWaitTargetStep = 0;

volatile int64_t beltStepCounter = 0;
volatile uint32_t beltPulseAccumulator = 0;

volatile uint32_t sorterStartedCount = 0;
volatile uint32_t sorterLateDropCount = 0;
volatile uint32_t sorterCmdDropCount = 0;
volatile uint32_t irDropCount = 0;

uint8_t rxBuffer[RX_BUFFER_MAX];
int rxIndex = 0;
int expectedPacketSize = 0;

bool irLastState = false;
unsigned long irLastTriggerMs = 0;

// ==================== IR event map ====================
struct PendingIrEvent {
  int32_t eventId;
  int64_t targetStep;
  bool active;
};

const int IR_EVENT_BUFFER_SIZE = 16;
PendingIrEvent pendingIrEvents[IR_EVENT_BUFFER_SIZE];
volatile int32_t nextEventId = 1;

// ==================== Sorter command queue ====================
struct SorterCommand {
  int32_t eventId;
  int64_t targetStep;
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

void emitOneBeltStep() {
  digitalWrite(BELT_STP_PIN, HIGH);
  delayMicroseconds(BELT_STEP_PULSE_US);
  digitalWrite(BELT_STP_PIN, LOW);
}

void setBeltTargetHz(unsigned int hz) {
  if (hz > 0 && hz < BELT_MIN_HZ) hz = BELT_MIN_HZ;
  if (hz > BELT_MAX_HZ) hz = BELT_MAX_HZ;
  beltTargetHz = hz;
}

float getMaxPendingIrProgress(bool* hasPendingOut) {
  bool hasPending = false;
  float maxProgress = 0.0f;

  noInterrupts();
  int64_t beltNow = beltStepCounter;
  for (int i = 0; i < IR_EVENT_BUFFER_SIZE; i++) {
    if (!pendingIrEvents[i].active) {
      continue;
    }

    hasPending = true;
    int64_t startStep = pendingIrEvents[i].targetStep - (int64_t)IR_TO_SORTER_DISTANCE_STEPS;
    int64_t traveled = beltNow - startStep;
    float progress = (float)traveled / (float)IR_TO_SORTER_DISTANCE_STEPS;
    if (progress > maxProgress) {
      maxProgress = progress;
    }
  }
  interrupts();

  if (maxProgress < 0.0f) maxProgress = 0.0f;
  if (maxProgress > 1.0f) maxProgress = 1.0f;

  *hasPendingOut = hasPending;
  return maxProgress;
}

void applyApiDelayCompensation() {
  bool hasPending = false;
  float maxProgress = getMaxPendingIrProgress(&hasPending);

  unsigned int baseHz;
  noInterrupts();
  baseHz = runtimeBeltStepsPerSec;
  interrupts();

  unsigned int targetHz = baseHz;
  if (hasPending) {
    if (maxProgress >= 0.8f) {
      targetHz = 0;
    } else if (maxProgress >= 0.5f) {
      targetHz = baseHz / 2;
    }
  }

  setBeltTargetHz(targetHz);
}

void updateBeltRamp() {
  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - beltRampLastMs;
  if (elapsedMs < BELT_RAMP_INTERVAL_MS) {
    return;
  }

  unsigned long tickCount = elapsedMs / BELT_RAMP_INTERVAL_MS;
  beltRampLastMs += tickCount * BELT_RAMP_INTERVAL_MS;

  unsigned int currentHz = beltCurrentHz;
  unsigned int targetHz = beltTargetHz;

  if (currentHz == targetHz) {
    return;
  }

  unsigned long maxDelta = tickCount * BELT_RAMP_STEP_HZ;

  if (currentHz < targetHz) {
    unsigned long nextHz = (unsigned long)currentHz + maxDelta;
    beltCurrentHz = (nextHz >= targetHz) ? targetHz : (unsigned int)nextHz;
  } else {
    unsigned long current = currentHz;
    beltCurrentHz = (maxDelta >= current - targetHz)
      ? targetHz
      : (unsigned int)(current - maxDelta);
  }
}

int findPendingIrEventSlot(int32_t eventId) {
  for (int i = 0; i < IR_EVENT_BUFFER_SIZE; i++) {
    if (pendingIrEvents[i].active && pendingIrEvents[i].eventId == eventId) {
      return i;
    }
  }
  return -1;
}

int allocPendingIrEventSlot() {
  for (int i = 0; i < IR_EVENT_BUFFER_SIZE; i++) {
    if (!pendingIrEvents[i].active) {
      return i;
    }
  }
  return -1;
}

bool consumePendingIrTargetStep(int32_t eventId, int64_t* targetStepOut) {
  int slot = findPendingIrEventSlot(eventId);
  if (slot < 0) {
    return false;
  }

  *targetStepOut = pendingIrEvents[slot].targetStep;
  pendingIrEvents[slot].active = false;
  return true;
}

bool enqueueSorterCommand(int32_t eventId, int64_t targetStep) {
  if (cmdQCount >= CMD_QUEUE_SIZE) {
    sorterCmdDropCount++;
    return false;
  }

  SorterCommand cmd;
  cmd.eventId = eventId;
  cmd.targetStep = targetStep;
  cmdQueue[cmdQTail] = cmd;
  cmdQTail = (cmdQTail + 1) % CMD_QUEUE_SIZE;
  cmdQCount++;
  return true;
}

bool dequeueSorterCommand(SorterCommand* outCmd) {
  if (cmdQCount <= 0) {
    return false;
  }

  *outCmd = cmdQueue[cmdQHead];
  cmdQHead = (cmdQHead + 1) % CMD_QUEUE_SIZE;
  cmdQCount--;
  return true;
}

void startSorterStrike(unsigned long nowUs) {
  sorterStartTimeUs = nowUs;
  sorterDurationUs = runtimeTFixed * 1000000.0f;
  sorterCurrentStep = 0;
  digitalWrite(SORTER_DIR_PIN, HIGH);
  sorterState = SORTER_STRIKE;
  sorterStartedCount++;
}

void startNextFromQueue() {
  while (cmdQCount > 0) {
    SorterCommand cmd;
    if (!dequeueSorterCommand(&cmd)) {
      return;
    }

    if (beltStepCounter >= cmd.targetStep) {
      sorterLateDropCount++;
      continue;
    }

    sorterWaitTargetStep = cmd.targetStep;
    sorterState = SORTER_WAIT_STEP;
    return;
  }
}

bool createIrEventAndNotify() {
  int32_t eventId;
  int64_t targetStep;
  bool created = false;

  noInterrupts();

  int slot = allocPendingIrEventSlot();
  if (slot >= 0) {
    eventId = nextEventId;
    nextEventId++;
    if (nextEventId <= 0) {
      nextEventId = 1;
    }

    targetStep = beltStepCounter + (int64_t)IR_TO_SORTER_DISTANCE_STEPS;

    pendingIrEvents[slot].eventId = eventId;
    pendingIrEvents[slot].targetStep = targetStep;
    pendingIrEvents[slot].active = true;
    created = true;
  } else {
    irDropCount++;
  }

  interrupts();

  if (!created) {
    Serial.println("IR_DROP");
    return false;
  }

  Serial.print("IR_DETECTED:");
  Serial.println(eventId);
  return true;
}

void timerISR(timer_callback_args_t __attribute((unused)) *args) {
  unsigned int hz = beltCurrentHz;
  beltPulseAccumulator += (uint32_t)hz * (uint32_t)TIMER_PERIOD_US;

  while (beltPulseAccumulator >= 1000000UL) {
    emitOneBeltStep();
    beltPulseAccumulator -= 1000000UL;
    beltStepCounter++;
  }

  unsigned long nowUs = micros();

  switch (sorterState) {
    case SORTER_IDLE:
      break;

    case SORTER_WAIT_STEP: {
      if (beltStepCounter >= sorterWaitTargetStep) {
        startSorterStrike(nowUs);
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
  int32_t eventId = 0;
  int32_t classifyFlag = 0;
  memcpy(&eventId, payload, 4);
  memcpy(&classifyFlag, payload + 4, 4);

  if (eventId <= 0) {
    return;
  }

  if (classifyFlag == 0) {
    noInterrupts();
    int64_t ignored;
    consumePendingIrTargetStep(eventId, &ignored);
    interrupts();
    return;
  }

  if (classifyFlag != 1) {
    return;
  }

  int64_t targetStep = 0;

  noInterrupts();

  bool found = consumePendingIrTargetStep(eventId, &targetStep);
  if (!found) {
    interrupts();
    return;
  }

  if (beltStepCounter >= targetStep) {
    sorterLateDropCount++;
    interrupts();
    return;
  }

  bool queued = enqueueSorterCommand(eventId, targetStep);
  if (queued && sorterState == SORTER_IDLE) {
    startNextFromQueue();
  }

  interrupts();
}

void handleTestPayload(const uint8_t* payload, int payloadSize) {
  (void)payload;
  if (payloadSize != 0) {
    return;
  }
  createIrEventAndNotify();
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
  memcpy(&tFixed, payload + 4, 4);
  memcpy(&tHold, payload + 8, 4);
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
  if (packetSize < 4) return;
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
  } else if (type == PACKET_TYPE_TEST && payloadSize == TEST_PAYLOAD_SIZE) {
    handleTestPayload(payload, payloadSize);
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
    } else if (type == PACKET_TYPE_TEST) {
      expectedPacketSize = TEST_PACKET_SIZE;
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

  sorterState = SORTER_IDLE;

  for (int i = 0; i < IR_EVENT_BUFFER_SIZE; i++) {
    pendingIrEvents[i].active = false;
  }

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
      createIrEventAndNotify();
      irLastTriggerMs = nowMs;
    }
  }
  irLastState = irCurrentState;

  while (Serial.available() > 0) {
    uint8_t inByte = Serial.read();
    parseIncomingByte(inByte);
  }
}
