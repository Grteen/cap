// ============================================================================
// calibration.ino
// 벨트 기준 거리 보정을 위한 독립 캘리브레이션 펌웨어
// ============================================================================

#include <Arduino.h>
#include <stdint.h>

// 핀 설정 (기존 시스템과 동일)
const int BELT_STP_PIN = 11;
const int BELT_DIR_PIN = 12;
const int BELT_EN_PIN  = 13;

const unsigned long SERIAL_BAUD_RATE = 115200;
const uint8_t FRAME_START = 0xAA;
const uint8_t FRAME_END = 0xFF;

// 명령 타입
const uint8_t CMD_JOG = 0x10;
const uint8_t CMD_ZERO = 0x11;
const uint8_t CMD_GET_POS = 0x12;
const uint8_t CMD_SET_DISTANCE = 0x13;
const uint8_t CMD_GET_DISTANCE = 0x14;

// 응답 타입
const uint8_t RSP_ACK = 0x90;
const uint8_t RSP_ERR = 0x91;

// 오류 코드
const uint8_t ERR_UNKNOWN_CMD = 1;
const uint8_t ERR_BAD_LENGTH = 2;
const uint8_t ERR_BAD_CHECKSUM = 3;

const uint8_t RX_BUFFER_MAX = 64;
const unsigned int BELT_STEP_PULSE_US = 8;
const unsigned int JOG_STEP_INTERVAL_US = 700;

int32_t currentPosSteps = 0;
int32_t savedDistanceSteps = 0;

uint8_t rxBuffer[RX_BUFFER_MAX];
int rxIndex = 0;
int expectedFrameSize = 0;

uint8_t computeChecksum(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t cs = frameType ^ payloadLen;
  for (uint8_t i = 0; i < payloadLen; i++) {
    cs ^= payload[i];
  }
  return cs;
}

void resetRxState() {
  rxIndex = 0;
  expectedFrameSize = 0;
}

void writeInt32LE(uint8_t* out, int32_t value) {
  memcpy(out, &value, 4);
}

void sendFrame(uint8_t frameType, const uint8_t* payload, uint8_t payloadLen) {
  uint8_t cs = computeChecksum(frameType, payload, payloadLen);
  Serial.write(FRAME_START);
  Serial.write(frameType);
  Serial.write(payloadLen);
  for (uint8_t i = 0; i < payloadLen; i++) {
    Serial.write(payload[i]);
  }
  Serial.write(cs);
  Serial.write(FRAME_END);
}

void sendAck(uint8_t cmdType) {
  uint8_t payload[9];
  payload[0] = cmdType;
  writeInt32LE(payload + 1, currentPosSteps);
  writeInt32LE(payload + 5, savedDistanceSteps);
  sendFrame(RSP_ACK, payload, 9);
}

void sendErr(uint8_t cmdType, uint8_t errCode) {
  uint8_t payload[2];
  payload[0] = cmdType;
  payload[1] = errCode;
  sendFrame(RSP_ERR, payload, 2);
}

void emitOneBeltStep() {
  digitalWrite(BELT_STP_PIN, HIGH);
  delayMicroseconds(BELT_STEP_PULSE_US);
  digitalWrite(BELT_STP_PIN, LOW);
}

void jogBeltSteps(int32_t delta) {
  if (delta == 0) {
    return;
  }

  bool forward = (delta > 0);
  uint32_t steps = (delta > 0) ? (uint32_t)delta : (uint32_t)(-delta);

  digitalWrite(BELT_DIR_PIN, forward ? HIGH : LOW);

  for (uint32_t i = 0; i < steps; i++) {
    emitOneBeltStep();
    if (forward) {
      currentPosSteps++;
    } else {
      currentPosSteps--;
    }
    delayMicroseconds(JOG_STEP_INTERVAL_US);
  }
}

void processCommand(uint8_t cmdType, const uint8_t* payload, uint8_t payloadLen) {
  if (cmdType == CMD_JOG) {
    if (payloadLen != 4) {
      sendErr(cmdType, ERR_BAD_LENGTH);
      return;
    }
    int32_t delta = 0;
    memcpy(&delta, payload, 4);
    jogBeltSteps(delta);
    sendAck(cmdType);
    return;
  }

  if (cmdType == CMD_ZERO) {
    if (payloadLen != 0) {
      sendErr(cmdType, ERR_BAD_LENGTH);
      return;
    }
    currentPosSteps = 0;
    sendAck(cmdType);
    return;
  }

  if (cmdType == CMD_GET_POS) {
    if (payloadLen != 0) {
      sendErr(cmdType, ERR_BAD_LENGTH);
      return;
    }
    sendAck(cmdType);
    return;
  }

  if (cmdType == CMD_SET_DISTANCE) {
    if (payloadLen != 0) {
      sendErr(cmdType, ERR_BAD_LENGTH);
      return;
    }
    savedDistanceSteps = currentPosSteps;
    sendAck(cmdType);
    return;
  }

  if (cmdType == CMD_GET_DISTANCE) {
    if (payloadLen != 0) {
      sendErr(cmdType, ERR_BAD_LENGTH);
      return;
    }
    sendAck(cmdType);
    return;
  }

  sendErr(cmdType, ERR_UNKNOWN_CMD);
}

void processFrame(const uint8_t* frame, int frameSize) {
  if (frameSize < 5) {
    return;
  }

  if (frame[0] != FRAME_START || frame[frameSize - 1] != FRAME_END) {
    return;
  }

  uint8_t cmdType = frame[1];
  uint8_t payloadLen = frame[2];
  int expected = 1 + 1 + 1 + payloadLen + 1 + 1;
  if (expected != frameSize) {
    return;
  }

  const uint8_t* payload = frame + 3;
  uint8_t recvChecksum = frame[frameSize - 2];
  uint8_t calcChecksum = computeChecksum(cmdType, payload, payloadLen);
  if (recvChecksum != calcChecksum) {
    sendErr(cmdType, ERR_BAD_CHECKSUM);
    return;
  }

  processCommand(cmdType, payload, payloadLen);
}

void parseIncomingByte(uint8_t inByte) {
  if (rxIndex == 0) {
    if (inByte == FRAME_START) {
      rxBuffer[rxIndex++] = inByte;
    }
    return;
  }

  if (rxIndex == 1) {
    rxBuffer[rxIndex++] = inByte;
    return;
  }

  if (rxIndex == 2) {
    rxBuffer[rxIndex++] = inByte;
    uint8_t payloadLen = inByte;
    expectedFrameSize = 1 + 1 + 1 + payloadLen + 1 + 1;
    if (expectedFrameSize > RX_BUFFER_MAX) {
      resetRxState();
    }
    return;
  }

  if (rxIndex >= RX_BUFFER_MAX) {
    resetRxState();
    return;
  }

  rxBuffer[rxIndex++] = inByte;
  if (expectedFrameSize > 0 && rxIndex >= expectedFrameSize) {
    processFrame(rxBuffer, expectedFrameSize);
    resetRxState();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(BELT_STP_PIN, OUTPUT);
  pinMode(BELT_DIR_PIN, OUTPUT);
  pinMode(BELT_EN_PIN, OUTPUT);

  // 드라이버 활성화(LOW)
  digitalWrite(BELT_EN_PIN, LOW);
  digitalWrite(BELT_STP_PIN, LOW);
  digitalWrite(BELT_DIR_PIN, HIGH);
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t inByte = (uint8_t)Serial.read();
    parseIncomingByte(inByte);
  }
}

