# AI Conveyor Sorting System

## 1. 개요

- IR 센서 감지 시 Python이 카메라 프레임을 캡처하고 Gemini로 분류(`0/1`)합니다.
- Python이 ETA를 계산해 Arduino로 이벤트 패킷을 전송합니다.
- Arduino는 5차 다항식 궤적으로 소터를 구동하고 벨트는 `tone()` 기반으로 동작합니다.
- 대시보드(`dashboard.py`)에서 라이브 영상, AI 응답, ETA, 지연 시간, 큐 상태, 하드웨어 설정을 확인/변경합니다.

## 2. 구성 파일

- `main.py`: 하드웨어 제어, AI 분류, 시리얼 프로토콜, 설정 동기화
- `ohlets.ino`: 모터 제어 ISR, 패킷 파싱, ACK/에러 응답, IR 감지
- `dashboard.py`: PyQt6 대시보드 UI + 설정 다이얼로그
- `config/hw_params.json`: 런타임 설정 저장 파일(자동 생성/갱신)

## 3. 실행 및 빌드

### Python 실행

```powershell
uv run main.py
```

또는:

```powershell
uv run start
```

### 환경 변수

`.env`:

```env
GOOGLE_CLOUD_API_KEY=your_key_here
```

### Arduino 컴파일/업로드 (arduino-cli.exe)

```powershell
.\arduino-cli.exe compile --fqbn arduino:renesas_uno:unor4wifi .
.\arduino-cli.exe upload -p COM3 --fqbn arduino:renesas_uno:unor4wifi .
```

포트는 실제 연결 포트로 변경하세요.

## 4. 핵심 상수 (현재 코드값)

### Python (`main.py`)

- `IR_TO_SORTER_DISTANCE_STEPS = 10000`
- `BELT_STEPS_PER_SECOND = 2000`
- `SORTER_TARGET_DEGREE = 90.0`
- `MOTOR_FULL_STEPS_PER_REV = 200`
- `MOTOR_MICROSTEP_SETTING = 16`
- `SERIAL_PORT = 'COM3'`
- `BAUD_RATE = 115200`
- `CAMERA_INDEX = 0`
- `AI_MODEL_NAME = 'gemini-3.1-flash-lite-preview'`

런타임 기본값(`HardwareConfig.defaults()`):
- `belt_steps_per_sec = 2000`
- `t_fixed = 0.4`
- `t_hold = 0.8`
- `t_return = 0.4`
- `ir_debounce_ms = 500`

### Arduino (`ohlets.ino`)

- 핀: `SORTER(10/9/8), BELT(11/12/13), IR(7)`
- `SERIAL_BAUD_RATE = 115200`
- `TIMER_PERIOD_US = 100`
- `SORTER_STEP_PULSE_US = 8`
- `BELT_MIN_HZ = 100`, `BELT_MAX_HZ = 10000`
- 런타임 기본값:
  - `runtimeBeltStepsPerSec = 2000`
  - `runtimeTFixed = 0.4`
  - `runtimeTHold = 0.8`
  - `runtimeTReturn = 0.4`
  - `runtimeIrDebounceMs = 500`

## 5. 통신 프로토콜 (최신)

공통 프레임:

```text
[START=0xAA][TYPE][PAYLOAD...][CHECKSUM][END=0xFF]
```

- 체크섬: `type XOR payload 모든 바이트`
- 엔디안: Little-endian (`struct.pack('<...')`)

### 패킷 타입

- `TYPE=0x01` EVENT, payload `'<if'` (8B)
  - `classify_flag(int32)`, `t_eta(float32)`
- `TYPE=0x02` COEF, payload `'<6f'` (24B)
  - `c0..c5(float32)`
- `TYPE=0x03` CFG, payload `'<4fI'` (20B)
  - `belt_steps_per_sec, t_fixed, t_hold, t_return, ir_debounce_ms`

총 패킷 길이:
- EVENT: 12B
- COEF: 28B
- CFG: 24B

### ACK/상태 문자열

- Arduino → Python:
  - `IR_DETECTED`
  - `CFG_OK`, `CFG_BUSY`, `CFG_ERR`
  - `COEF_OK`, `COEF_ERR`

## 6. Constants Sync 체크리스트

아래는 수정 시 반드시 같이 확인해야 하는 항목입니다.

1. Baud rate
- `main.py: BAUD_RATE` ↔ `ohlets.ino: SERIAL_BAUD_RATE` (현재 `115200`)

2. 패킷 시작/종료 바이트
- `main.py: PACKET_START/PACKET_END` ↔ `ohlets.ino: PACKET_START_BYTE/PACKET_END_BYTE` (`0xAA/0xFF`)

3. 패킷 타입 값
- `EVENT=0x01`, `COEF=0x02`, `CFG=0x03` 양쪽 동일 필수

4. 벨트 속도 범위/기본값
- Python 설정 범위 `100~10000`과 Arduino `BELT_MIN_HZ/BELT_MAX_HZ` 일치 필요
- 기본값 `2000` 일치 필요

5. 런타임 타이밍 파라미터 범위
- Python 검증: `t_fixed/t_hold/t_return = 0.1~5.0`, `ir_debounce_ms = 100~5000`
- Arduino 검증 로직 동일 범위 사용

6. 모터 스텝 계산 기준
- `SORTER_TARGET_DEGREE`, `MOTOR_FULL_STEPS_PER_REV`, `MOTOR_MICROSTEP_SETTING`
- 실제 드라이버 마이크로스텝 하드웨어 설정과 반드시 일치

7. API 키 이름
- 코드 기준은 `GOOGLE_CLOUD_API_KEY` (기존 문서의 `GEMINI_API_KEY` 아님)

## 7. 대시보드 동작 요약

- 연결 LED: 카메라/아두이노/API 상태 표시
- 처리 대기열: AI 비동기 작업 수 표시
- 테스트 버튼: IR 이벤트 시뮬레이션
- 하드웨어 설정 다이얼로그:
  - 저장 시 Arduino에 CFG 전송 + COEF 재동기화
  - 성공 시 `config/hw_params.json` 저장 및 UI 값 갱신
  - 소터 동작 중이면 `CFG_BUSY`로 거부 가능

## 8. 알려진 주의사항

- 카메라 초기화는 Windows DirectShow(`cv2.CAP_DSHOW`) 사용
- API 응답이 늦으면 ETA 부족으로 `classification_missed` 발생 가능
- 시리얼 포트 오픈 시 Arduino 리셋 대기로 `2.5초` 대기함
