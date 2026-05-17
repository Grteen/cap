import json
import os
import re
import struct
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import serial
from dotenv import load_dotenv
from google import genai
from google.genai import types
from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QApplication

from dashboard import DashboardApp

# ==================== 기본 상수 (Defaults) ====================
IR_TO_SORTER_DISTANCE_STEPS = 100000
BELT_STEPS_PER_SECOND = 2000
SORTER_TARGET_DEGREE = 70
MOTOR_FULL_STEPS_PER_REV = 200
MOTOR_MICROSTEP_SETTING = 16
SERIAL_PORT = 'COM4'        # 시리얼 포트
BAUD_RATE = 115200          
CAMERA_INDEX = 0            # 카메라 번호
AI_MODEL_NAME = 'gemini-3.1-flash-lite'     # AI 모델
DEFAULT_AI_PROMPT = (
    "Analyze this object. If the object color is closer to blue, response exactly with '0'. "
    "If it is closer to yellow, response exactly with '1'. Respond only with a single digit, either '1' or '0'."
)

CONFIG_PATH = Path('config/hw_params.json')

PACKET_START = 0xAA
PACKET_END = 0xFF
PACKET_TYPE_EVENT = 0x01
PACKET_TYPE_COEF = 0x02
PACKET_TYPE_CFG = 0x03
PACKET_TYPE_TEST_TRIGGER = 0x04

ACK_CFG_OK = 'CFG_OK'
ACK_CFG_BUSY = 'CFG_BUSY'
ACK_CFG_ERR = 'CFG_ERR'
ACK_COEF_OK = 'COEF_OK'
ACK_COEF_ERR = 'COEF_ERR'

# 시리얼 통신 동시 접근 방지
serial_lock = threading.Lock()


@dataclass
class HardwareConfig:
    belt_steps_per_sec: int
    t_fixed: float
    t_hold: float
    t_return: float
    ir_debounce_ms: int

    @classmethod
    def defaults(cls) -> 'HardwareConfig':
        return cls(
            belt_steps_per_sec=BELT_STEPS_PER_SECOND,
            t_fixed=0.4,
            t_hold=0.8,
            t_return=0.4,
            ir_debounce_ms=500,
        )

    @classmethod
    def from_dict(cls, data: dict) -> 'HardwareConfig':
        cfg = cls(
            belt_steps_per_sec=int(data['belt_steps_per_sec']),
            t_fixed=float(data['t_fixed']),
            t_hold=float(data['t_hold']),
            t_return=float(data['t_return']),
            ir_debounce_ms=int(data['ir_debounce_ms']),
        )
        cfg.validate()
        return cfg

    def validate(self):
        if not (100 <= self.belt_steps_per_sec <= 10000):
            raise ValueError('belt_steps_per_sec 범위는 100~10000 이어야 합니다.')
        for name, val in [
            ('t_fixed', self.t_fixed),
            ('t_hold', self.t_hold),
            ('t_return', self.t_return),
        ]:
            if not (0.1 <= val <= 5.0):
                raise ValueError(f'{name} 범위는 0.1~5.0 이어야 합니다.')
        if not (100 <= self.ir_debounce_ms <= 5000):
            raise ValueError('ir_debounce_ms 범위는 100~5000 이어야 합니다.')


def ensure_config_dir():
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)


def load_hw_config() -> HardwareConfig:
    ensure_config_dir()
    if not CONFIG_PATH.exists():
        cfg = HardwareConfig.defaults()
        save_hw_config(cfg)
        return cfg

    try:
        data = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
        return HardwareConfig.from_dict(data)
    except Exception as exc:
        print(f"설정 파일 로드 실패. 기본값 사용: {exc}")
        cfg = HardwareConfig.defaults()
        save_hw_config(cfg)
        return cfg


def save_hw_config(cfg: HardwareConfig):
    ensure_config_dir()
    CONFIG_PATH.write_text(
        json.dumps(asdict(cfg), ensure_ascii=False, indent=2),
        encoding='utf-8',
    )


def calculate_5th_polynomial_coeffs(target_steps):
    s_f = float(target_steps)
    c0 = 0.0
    c1 = 0.0
    c2 = 0.0
    c3 = 10.0 * s_f
    c4 = -15.0 * s_f
    c5 = 6.0 * s_f
    return c0, c1, c2, c3, c4, c5


def _checksum(packet_type: int, payload: bytes) -> int:
    csum = packet_type & 0xFF
    for b in payload:
        csum ^= b
    return csum & 0xFF


def build_packet(packet_type: int, payload: bytes) -> bytes:
    return bytes([
        PACKET_START,
        packet_type,
        *payload,
        _checksum(packet_type, payload),
        PACKET_END,
    ])


def build_event_packet(event_id: int, classify_flag: int) -> bytes:
    payload = struct.pack('<ii', int(event_id), int(classify_flag))
    return build_packet(PACKET_TYPE_EVENT, payload)


def build_coeff_packet(coeffs: tuple) -> bytes:
    payload = struct.pack('<6f', *[float(x) for x in coeffs])
    return build_packet(PACKET_TYPE_COEF, payload)


def build_cfg_packet(cfg: HardwareConfig) -> bytes:
    payload = struct.pack(
        '<4fI',
        float(cfg.belt_steps_per_sec),
        float(cfg.t_fixed),
        float(cfg.t_hold),
        float(cfg.t_return),
        int(cfg.ir_debounce_ms),
    )
    return build_packet(PACKET_TYPE_CFG, payload)


def build_test_trigger_packet() -> bytes:
    return build_packet(PACKET_TYPE_TEST_TRIGGER, b'')


def init_hardware():
    print(f"[{time.strftime('%H:%M:%S')}] 카메라({CAMERA_INDEX})를 초기화 중입니다...")
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if cap.isOpened():
        # Force strict 16:9 widescreen HD resolution to eliminate any black bars
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"[카메라 설정] 해상도가 {w}x{h} (비율: {w/h if h > 0 else 0:.2f})로 성공적으로 로드되었습니다.")
    else:
        print(f"경고: 카메라 {CAMERA_INDEX}를 열 수 없습니다.")

    print(f"[{time.strftime('%H:%M:%S')}] 시리얼 포트({SERIAL_PORT}) 연결 시도 중...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        print('시리얼 포트 연결 완료. 아두이노 리셋 대기 중...')
        time.sleep(2.5)
        ser.reset_input_buffer()
        print('아두이노 준비 완료.')
    except Exception as e:
        print(f'시리얼 연결 오류: {e}')
        ser = None

    return cap, ser


def classify_image_with_ai(client: genai.Client, frame, prompt_text: str):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    success, encoded_img = cv2.imencode('.jpg', frame, encode_param)

    if not success:
        print('이미지 인코딩에 실패했습니다.')
        return 0, '이미지 인코딩 실패'

    image_bytes = encoded_img.tobytes()
    try:
        print('AI 모델로 이미지 전송 중...')

        generate_content_config = types.GenerateContentConfig(
            thinking_config=types.ThinkingConfig(
                thinking_level='MINIMAL',
            ),
        )

        response = client.models.generate_content(
            model=AI_MODEL_NAME,
            contents=[
                types.Part.from_bytes(
                    data=image_bytes,
                    mime_type='image/jpeg',
                ),
                prompt_text,
            ],
            config=generate_content_config,
        )

        result_text = response.text.strip()
        print(f'AI 응답 결과: {result_text}')

        # [] 내부 숫자를 우선 파싱한다. 예: "[1]", "판정: [0]"
        match = re.search(r'\[\s*(-?\d+)\s*\]', result_text)
        if match:
            parsed_num = int(match.group(1))
            return (1 if parsed_num == 1 else 0), result_text

        # 하위 호환: 기존 단일 숫자 응답도 허용
        if result_text == '1':
            return 1, result_text
        if result_text == '0':
            return 0, result_text
        return 0, result_text
    except Exception as e:
        print(f'AI API 통신 중 오류 발생: {e}')
        return 0, f'[오류] {e}'


def send_event_to_arduino(ser: serial.Serial, event_id: int, classify_flag: int):
    if ser is None or not ser.is_open:
        return

    packet = build_event_packet(event_id, classify_flag)
    with serial_lock:
        ser.write(packet)

    print(f"[{time.strftime('%H:%M:%S')}] EVT 전송 (event_id={event_id}, flag={classify_flag})")


def send_test_trigger_to_arduino(ser: serial.Serial):
    if ser is None or not ser.is_open:
        return False

    with serial_lock:
        ser.write(build_test_trigger_packet())
    return True


class HardwareController(QThread):
    frame_ready = pyqtSignal(object)
    ai_image_ready = pyqtSignal(object)
    classification_started = pyqtSignal(int, str)
    classification_finished = pyqtSignal(int, int, float, tuple, str)
    connection_status = pyqtSignal(bool, bool)
    api_status = pyqtSignal(bool)
    queue_updated = pyqtSignal(int)
    settings_apply_result = pyqtSignal(bool, str, dict)

    def __init__(self, target_steps, client, hw_config: HardwareConfig, run_folder: str = ""):
        super().__init__()
        self.target_steps = target_steps
        self.client = client
        self.running = True
        self.run_folder = run_folder
        self._ir_events = deque()
        self._ir_lock = threading.Lock()
        self.cap = None
        self.ser = None
        self._pending_count = 0
        self._count_lock = threading.Lock()
        self._ai_semaphore = threading.Semaphore(12)
        self.hw_config = hw_config
        self.prompt_text = DEFAULT_AI_PROMPT
        self.cached_coeffs = calculate_5th_polynomial_coeffs(target_steps)

        self._config_lock = threading.Lock()
        self._pending_config_dict = None
        self._virtual_delay_lock = threading.Lock()
        self.virtual_delay_sec = 0.0
        self._simulated_event_counter = 0

    def _enqueue_ir_event(self, event_id: int):
        with self._ir_lock:
            self._ir_events.append(event_id)

    def _dequeue_ir_event(self):
        with self._ir_lock:
            if not self._ir_events:
                return None
            return self._ir_events.popleft()

    @staticmethod
    def _parse_ir_event_id(line: str):
        if not line.startswith('IR_DETECTED'):
            return None
        _, sep, tail = line.partition(':')
        if not sep:
            return None
        try:
            return int(tail.strip())
        except ValueError:
            return None

    def trigger_test_event(self):
        ok = send_test_trigger_to_arduino(self.ser)
        if not ok:
            print('시리얼 포트 미연결: 대시보드 가상 IR 센서 이벤트를 즉시 강제 트리거합니다.')
            self._simulated_event_counter += 1
            self._enqueue_ir_event(self._simulated_event_counter)

    @pyqtSlot(str)
    def update_prompt(self, prompt_text: str):
        prompt_text = (prompt_text or '').strip()
        if prompt_text:
            self.prompt_text = prompt_text
            print('AI 프롬프트가 대시보드에서 업데이트되었습니다.')

    @pyqtSlot(dict)
    def request_settings_apply(self, cfg_dict: dict):
        with self._config_lock:
            self._pending_config_dict = cfg_dict

    @pyqtSlot(float)
    def update_virtual_delay(self, delay_sec: float):
        delay_sec = max(0.0, float(delay_sec))
        with self._virtual_delay_lock:
            self.virtual_delay_sec = delay_sec
        print(f'가상 API 지연이 {delay_sec:.1f}초로 설정되었습니다.')

    def _send_packet_with_ack(self, packet: bytes, ok_ack: str, err_ack: str, busy_ack: str = '', retries: int = 3, timeout_sec: float = 0.2):
        if self.ser is None or not self.ser.is_open:
            return False, '시리얼 포트가 연결되어 있지 않습니다.'

        for attempt in range(1, retries + 1):
            with serial_lock:
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                deadline = time.time() + timeout_sec

                while time.time() < deadline:
                    if self.ser.in_waiting <= 0:
                        time.sleep(0.005)
                        continue

                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    event_id = self._parse_ir_event_id(line)
                    if event_id is not None:
                        self._enqueue_ir_event(event_id)
                        continue

                    if line == ok_ack:
                        return True, ok_ack

                    if busy_ack and line == busy_ack:
                        return False, busy_ack

                    if line == err_ack:
                        break

            print(f'ACK 대기 실패 ({attempt}/{retries})')

        return False, 'ACK_TIMEOUT'

    def _sync_arduino_startup(self):
        ok_cfg, msg_cfg = self._send_packet_with_ack(
            build_cfg_packet(self.hw_config),
            ok_ack=ACK_CFG_OK,
            err_ack=ACK_CFG_ERR,
            busy_ack=ACK_CFG_BUSY,
            retries=3,
            timeout_sec=0.3,
        )
        if not ok_cfg:
            print(f'초기 CFG 적용 실패: {msg_cfg}')
            return False

        ok_coef, msg_coef = self._send_packet_with_ack(
            build_coeff_packet(self.cached_coeffs),
            ok_ack=ACK_COEF_OK,
            err_ack=ACK_COEF_ERR,
            retries=3,
            timeout_sec=0.3,
        )
        if not ok_coef:
            print(f'초기 COEF 적용 실패: {msg_coef}')
            return False

        print('초기 CFG/COEF 동기화 완료')
        return True

    def _process_pending_settings_if_any(self):
        pending = None
        with self._config_lock:
            if self._pending_config_dict is not None:
                pending = self._pending_config_dict
                self._pending_config_dict = None

        if pending is None:
            return

        try:
            new_cfg = HardwareConfig.from_dict(pending)
        except Exception as exc:
            self.settings_apply_result.emit(False, f'입력값 검증 실패: {exc}', {})
            return

        ok_cfg, msg_cfg = self._send_packet_with_ack(
            build_cfg_packet(new_cfg),
            ok_ack=ACK_CFG_OK,
            err_ack=ACK_CFG_ERR,
            busy_ack=ACK_CFG_BUSY,
            retries=3,
            timeout_sec=0.25,
        )
        if not ok_cfg:
            if msg_cfg == ACK_CFG_BUSY:
                self.settings_apply_result.emit(False, '소터 동작 중이라 설정 적용을 거부했습니다. 잠시 후 다시 시도하세요.', {})
            else:
                self.settings_apply_result.emit(False, '설정 적용 ACK를 받지 못했습니다. 케이블/포트 상태를 확인하세요.', {})
            return

        ok_coef, msg_coef = self._send_packet_with_ack(
            build_coeff_packet(self.cached_coeffs),
            ok_ack=ACK_COEF_OK,
            err_ack=ACK_COEF_ERR,
            retries=3,
            timeout_sec=0.25,
        )
        if not ok_coef:
            self.settings_apply_result.emit(False, f'설정은 반영됐지만 계수 재동기화 실패: {msg_coef}', asdict(new_cfg))
            self.hw_config = new_cfg
            return

        self.hw_config = new_cfg
        self.settings_apply_result.emit(True, '하드웨어 설정 적용 완료', asdict(new_cfg))

    def process_classification_task(self, client, frame_copy, start_time, ser, event_id: int):
        try:
            with self._ai_semaphore:
                classify_flag = 0
                raw_text = ''
                if client and frame_copy is not None:
                    with self._virtual_delay_lock:
                        local_virtual_delay = self.virtual_delay_sec
                        self.virtual_delay_sec = 0.0
                    if local_virtual_delay > 0.0:
                        time.sleep(local_virtual_delay)
                    classify_flag, raw_text = classify_image_with_ai(client, frame_copy, self.prompt_text)
                else:
                    import random

                    delay = random.uniform(8.0, 15.0) if random.random() < 0.3 else 1.2
                    time.sleep(delay)
                    classify_flag = 1
                    raw_text = '1'

            api_delay = time.time() - start_time
            print(f'AI 모델 예측 및 API 왕복 지연 시간: {api_delay:.3f} 초')

            if self.run_folder:
                try:
                    event_folder = os.path.join(self.run_folder, str(event_id))
                    os.makedirs(event_folder, exist_ok=True)
                    if frame_copy is not None:
                        img_path = os.path.join(event_folder, "image.jpg")
                        cv2.imwrite(img_path, frame_copy)
                    res_path = os.path.join(event_folder, "response.txt")
                    with open(res_path, "w", encoding="utf-8") as f:
                        f.write(raw_text)
                    print(f"[로컬 아카이브 저장 완료] ID {event_id}: {event_folder}")
                except Exception as e:
                    print(f"[로컬 아카이브 저장 실패] {e}")

            send_event_to_arduino(ser, event_id, classify_flag)
            self.classification_finished.emit(event_id, classify_flag, api_delay, self.cached_coeffs, raw_text)
        finally:
            with self._count_lock:
                self._pending_count -= 1
                count = self._pending_count
            self.queue_updated.emit(count)

    def _ping_api(self):
        if self.client is None:
            self.api_status.emit(False)
            return
        try:
            print(f"[{time.strftime('%H:%M:%S')}] AI API 연결 상태 확인 중...")
            self.client.models.generate_content(
                model=AI_MODEL_NAME,
                contents=['ping'],
            )
            print(f"[{time.strftime('%H:%M:%S')}] AI API 연결 확인 완료.")
            self.api_status.emit(True)
        except Exception as e:
            print(f"[{time.strftime('%H:%M:%S')}] AI API 연결 실패: {e}")
            self.api_status.emit(False)

    def run(self):
        self.cap, self.ser = init_hardware()
        cam_ok = self.cap is not None and self.cap.isOpened()
        ser_ok = self.ser is not None and self.ser.is_open
        self.connection_status.emit(cam_ok, ser_ok)

        if ser_ok:
            self._sync_arduino_startup()

        threading.Thread(target=self._ping_api, daemon=True).start()

        print(f"[{time.strftime('%H:%M:%S')}] 하드웨어 컨트롤러 루프를 시작합니다.")

        while self.running:
            self._process_pending_settings_if_any()

            ret, frame = self.cap.read() if self.cap and self.cap.isOpened() else (False, None)
            if ret and frame is not None:
                self.frame_ready.emit(frame)

            if self.ser and self.ser.in_waiting > 0:
                try:
                    signal = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    event_id = self._parse_ir_event_id(signal)
                    if event_id is not None:
                        self._enqueue_ir_event(event_id)
                except Exception:
                    pass

            event_id = self._dequeue_ir_event()
            if event_id is not None:
                start_time_str = time.strftime('%H:%M:%S')
                print(f"\n[{start_time_str}] IR 감지(event_id={event_id}). 비동기 처리 시작")
                start_time = time.time()
                self.classification_started.emit(event_id, start_time_str)

                if not ret or frame is None:
                    print('사진 캡처 실패. 더미 데이터로 진행')
                    frame_to_process = None
                else:
                    frame_to_process = frame.copy()
                    self.ai_image_ready.emit(frame_to_process)

                with self._count_lock:
                    self._pending_count += 1
                    count = self._pending_count
                self.queue_updated.emit(count)

                threading.Thread(
                    target=self.process_classification_task,
                    args=(self.client, frame_to_process, start_time, self.ser, event_id),
                    daemon=True,
                ).start()

            time.sleep(0.01)

    def stop(self):
        self.running = False
        self.wait()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        if self.ser and self.ser.is_open:
            self.ser.close()


def main():
    load_dotenv()
    api_key = os.getenv('GOOGLE_CLOUD_API_KEY')
    if not api_key:
        print('경고: .env 파일에 GOOGLE_CLOUD_API_KEY가 없습니다.')

    client = genai.Client(vertexai=True, api_key=api_key) if api_key else None

    # Load persistent prompt from config if exists
    global DEFAULT_AI_PROMPT
    config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
    prompt_file = os.path.join(config_dir, "prompt.txt")
    if os.path.exists(prompt_file):
        try:
            with open(prompt_file, "r", encoding="utf-8") as f:
                saved_prompt = f.read().strip()
            if saved_prompt:
                DEFAULT_AI_PROMPT = saved_prompt
                print(f"[설정 로드] 저장되어 있던 AI 프롬프트를 불러왔습니다.")
        except Exception as e:
            print(f"저장된 프롬프트 로드 실패: {e}")

    hw_config = load_hw_config()
    steps_per_degree = (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEP_SETTING) / 360.0
    target_steps = int(SORTER_TARGET_DEGREE * steps_per_degree)
    print(f'설정된 목표 이동 스텝 수(종단점): {target_steps} steps')

    run_folder_name = f"run_{time.strftime('%Y%m%d_%H%M%S')}"
    base_dir = os.path.dirname(os.path.abspath(__file__))
    parent_folder = os.path.join(base_dir, "history")
    os.makedirs(parent_folder, exist_ok=True)
    run_folder_path = os.path.join(parent_folder, run_folder_name)
    os.makedirs(run_folder_path, exist_ok=True)
    print(f"[로컬 로그 아카이빙] 세션 런 폴더가 생성되었습니다: {run_folder_path}")

    app = QApplication([])
    dashboard = DashboardApp(initial_prompt=DEFAULT_AI_PROMPT, run_folder=run_folder_path)
    dashboard.set_hardware_info(
        SORTER_TARGET_DEGREE,
        hw_config.belt_steps_per_sec,
        MOTOR_MICROSTEP_SETTING,
        MOTOR_FULL_STEPS_PER_REV,
        IR_TO_SORTER_DISTANCE_STEPS,
        target_steps,
        SERIAL_PORT,
        BAUD_RATE,
        CAMERA_INDEX,
        AI_MODEL_NAME,
    )
    dashboard.set_runtime_settings(asdict(hw_config))
    dashboard.show()

    hw_thread = HardwareController(target_steps, client, hw_config, run_folder=run_folder_path)
    hw_thread.frame_ready.connect(dashboard.set_live_frame)
    hw_thread.ai_image_ready.connect(dashboard.set_ai_image)
    hw_thread.classification_started.connect(dashboard.add_classification_started)
    hw_thread.classification_finished.connect(dashboard.add_classification_result)
    hw_thread.connection_status.connect(dashboard.update_connection_status)
    hw_thread.api_status.connect(dashboard.update_api_status)
    hw_thread.queue_updated.connect(dashboard.update_queue_count)

    dashboard.test_ir_signal.connect(hw_thread.trigger_test_event)
    dashboard.settings_save_requested.connect(hw_thread.request_settings_apply)
    dashboard.prompt_save_requested.connect(hw_thread.update_prompt)
    dashboard.virtual_delay_changed.connect(hw_thread.update_virtual_delay)

    def handle_settings_result(success: bool, message: str, applied_values: dict):
        nonlocal hw_config
        if success and applied_values:
            try:
                hw_config = HardwareConfig.from_dict(applied_values)
                save_hw_config(hw_config)
                dashboard.set_hardware_info(
                    SORTER_TARGET_DEGREE,
                    hw_config.belt_steps_per_sec,
                    MOTOR_MICROSTEP_SETTING,
                    MOTOR_FULL_STEPS_PER_REV,
                    IR_TO_SORTER_DISTANCE_STEPS,
                    target_steps,
                    SERIAL_PORT,
                    BAUD_RATE,
                    CAMERA_INDEX,
                    AI_MODEL_NAME,
                )
                dashboard.set_runtime_settings(asdict(hw_config))
            except Exception as exc:
                message = f'{message} (로컬 저장 실패: {exc})'

        dashboard.handle_settings_apply_result(success, message, applied_values if success else None)

    hw_thread.settings_apply_result.connect(handle_settings_result)

    hw_thread.start()
    exit_code = app.exec()

    hw_thread.stop()
    print('\n어플리케이션을 안전하게 종료했습니다.')
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
