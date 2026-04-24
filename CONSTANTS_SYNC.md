# 상수 동기화 체크리스트 (Constants Sync Checklist)

파일 간 반드시 값이 일치해야 하는 상수들을 정리한 문서입니다.  
어느 한 쪽을 수정하면 대응되는 파일도 함께 수정해야 합니다.

---

## 🔴 CRITICAL — 불일치 시 통신/동작 즉시 오류

### 1. 시리얼 통신 속도 (Baud Rate)

| 파일 | 변수명 | 현재 값 |
|------|--------|---------|
| `main.py` | `BAUD_RATE` | `115200` |
| `ohlets.ino` | `SERIAL_BAUD_RATE` | `115200` |

> 불일치 시: 시리얼 통신 불가, 쓰레기 데이터 수신

---

### 2. 시리얼 패킷 구조

Python의 `struct.pack` 포맷과 Arduino의 `memcpy` 오프셋이 정확히 일치해야 합니다.

| 항목 | `main.py` (`send_coefficients_arduino`) | `ohlets.ino` (`parsePacket`) |
|------|----------------------------------------|------------------------------|
| 패킷 총 크기 | `struct.pack('<B i f f f f f f f B', ...)` = **34바이트** | `PACKET_SIZE = 34` |
| 시작 바이트 | `0xAA` | `PACKET_START_BYTE = 0xAA` |
| 종료 바이트 | `0xFF` | `PACKET_END_BYTE = 0xFF` |
| 엔디안 | `<` (리틀 엔디안) | ARM 아키텍처 = 리틀 엔디안 (자동 일치) |

**오프셋 구조 (변경 금지)**

```
[0]      : Start Byte (0xAA)         — 1 byte
[1~4]    : classify_flag (int32)     — 4 bytes
[5~8]    : ETA (float32)             — 4 bytes
[9~12]   : c0 (float32)              — 4 bytes
[13~16]  : c1 (float32)              — 4 bytes
[17~20]  : c2 (float32)              — 4 bytes
[21~24]  : c3 (float32)              — 4 bytes
[25~28]  : c4 (float32)              — 4 bytes
[29~32]  : c5 (float32)              — 4 bytes
[33]     : End Byte (0xFF)           — 1 byte
```

> 불일치 시: 분류 플래그/계수 값이 엉뚱한 바이트로 해석됨

---

### 3. 벨트 속도 (ETA 계산 기준)

| 파일 | 변수명 | 현재 값 | 역할 |
|------|--------|---------|------|
| `main.py` | `BELT_STEPS_PER_SECOND` | `2000` | ETA = 거리 ÷ 이 값 |
| `ohlets.ino` | `BELT_STEPS_PER_SEC` | `2000` | `tone()` 주파수 (Hz) = 실제 벨트 스텝 속도 |

> 불일치 시: Python이 계산한 ETA와 실제 물건 도착 시간이 어긋나 분류 타이밍 오류

---

## 🟠 IMPORTANT — 불일치 시 소터 각도/스텝 오류

### 4. 모터 스펙 (스텝 수 계산)

Python은 `target_steps = SORTER_TARGET_DEGREE × (MOTOR_FULL_STEPS_PER_REV × MOTOR_MICROSTEP_SETTING) / 360` 으로 스텝 수를 계산하여 Arduino에 5차 다항식 계수로 전달합니다.  
Arduino 드라이버의 실제 마이크로스텝 설정과 반드시 일치해야 합니다.

| 파일 | 변수명 | 현재 값 | 설명 |
|------|--------|---------|------|
| `main.py` | `MOTOR_FULL_STEPS_PER_REV` | `200` | 모터 1회전당 풀스텝 수 (1.8° 모터 기준) |
| `main.py` | `MOTOR_MICROSTEP_SETTING` | `16` | 드라이버 마이크로스텝 설정 |
| `ohlets.ino` | _(드라이버 하드웨어 핀 설정)_ | 16분주 | MS1/MS2/MS3 핀 물리 연결 값과 일치해야 함 |

> 불일치 시: 소터 팔이 목표 각도에 못 미치거나 초과함

---

### 5. 목표 각도

| 파일 | 변수명 | 현재 값 | 역할 |
|------|--------|---------|------|
| `main.py` | `SORTER_TARGET_DEGREE` | `90.0` | 스텝 수 계산에 사용, dashboard에도 표시됨 |
| `ohlets.ino` | _(직접 선언 없음)_ | — | Python이 계산한 스텝 수(`target_steps`)를 패킷 계수로 수신 |

> `SORTER_TARGET_DEGREE` 는 `main.py` 에서만 수정하면 됩니다. Arduino는 전달받은 스텝 수를 그대로 사용합니다.

---

## 🟡 ADVISORY — 불일치 시 타이밍 여유 감소

### 6. 소터 1사이클 시간 (물건 최소 간격 기준)

Arduino에서만 선언되며 Python은 이 값을 직접 참조하지 않습니다.  
단, **물건 최소 간격 설계 시 아래 합산값을 반드시 고려**해야 합니다.

| 파일 | 변수명 | 현재 값 | 설명 |
|------|--------|---------|------|
| `ohlets.ino` | `T_FIXED` | `0.4f` 초 | 팔 펼침(타격) 시간 |
| `ohlets.ino` | `T_HOLD` | `0.8f` 초 | 팔 유지 시간 |
| `ohlets.ino` | `T_RETURN` | `0.4f` 초 | 팔 복귀 시간 |

**1사이클 합계: 0.4 + 0.8 + 0.4 = 1.6초**

물건 간격이 1.6초보다 짧으면 Arduino 큐에 적재되며, ETA 보정 후 이미 통과한 물건은 자동 skip됩니다.  
실질적 안전 간격 = **1.6초 + AI API 평균 응답 지연**

---

### 7. IR 디바운스 시간

| 파일 | 변수명 | 현재 값 | 역할 |
|------|--------|---------|------|
| `ohlets.ino` | `IR_DEBOUNCE_MS` | `500` ms | Arduino IR 재감지 방지 |
| `main.py` | _(별도 없음)_ | — | Python은 Arduino로부터 `"IR_DETECTED"` 문자열 수신 |

> 디바운스가 500ms이면 초당 최대 2회 감지 가능. 벨트 속도와 물건 크기에 맞게 조정.

---

## 수정 시 체크 순서

```
하드웨어 변경 시:
  벨트 속도 변경     → main.py BELT_STEPS_PER_SECOND  +  ohlets.ino BELT_STEPS_PER_SEC
  마이크로스텝 변경  → main.py MOTOR_MICROSTEP_SETTING  +  드라이버 MS핀 물리 설정
  목표 각도 변경     → main.py SORTER_TARGET_DEGREE (Arduino는 자동 반영)
  소터 타이밍 변경   → ohlets.ino T_FIXED / T_HOLD / T_RETURN (물건 간격도 재검토)

통신 변경 시:
  Baud Rate 변경     → main.py BAUD_RATE  +  ohlets.ino SERIAL_BAUD_RATE
  패킷 구조 변경     → main.py struct.pack 포맷  +  ohlets.ino parsePacket memcpy 오프셋  (동시 수정 필수)
```
