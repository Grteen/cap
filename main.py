import json
import os
import re
import struct
import threading
import time
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

IR_TO_SORTER_DISTANCE_STEPS = 100000
BELT_STEPS_PER_SECOND = 2000
SORTER_TARGET_DEGREE = 45.0
MOTOR_FULL_STEPS_PER_REV = 200
MOTOR_MICROSTEP_SETTING = 16
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CAMERA_INDEX = 0
AI_MODEL_NAME = 'gemini-3.1-flash-lite-preview'
DEFAULT_AI_PROMPT = (
    "Analyze this object. If the object color is closer to blue, response exactly with '0'. "
    "If it is closer to yellow, response exactly with '1'. Respond only with a single digit, either '1' or '0'."
)

CONFIG_PATH = Path('config/hw_params.json')

PACKET_START = 0xAA
PACKET_END = 0xFF
PACKET_TYPE_CLASS_RESULT = 0x01
PACKET_TYPE_COEF = 0x02
PACKET_TYPE_CFG = 0x03

ACK_CFG_OK = 'CFG_OK'
ACK_CFG_BUSY = 'CFG_BUSY'
ACK_CFG_ERR = 'CFG_ERR'
ACK_COEF_OK = 'COEF_OK'
ACK_COEF_ERR = 'COEF_ERR'

serial_lock = threading.Lock()


@dataclass
class HardwareConfig:
    belt_steps_per_sec: int
    ir_to_sorter_distance_steps: int
    t_fixed: float
    t_hold: float
    t_return: float
    ir_debounce_ms: int

    @classmethod
    def defaults(cls) -> 'HardwareConfig':
        return cls(BELT_STEPS_PER_SECOND, IR_TO_SORTER_DISTANCE_STEPS, 0.4, 0.8, 0.4, 500)

    @classmethod
    def from_dict(cls, data: dict) -> 'HardwareConfig':
        cfg = cls(
            int(data['belt_steps_per_sec']),
            int(data.get('ir_to_sorter_distance_steps', IR_TO_SORTER_DISTANCE_STEPS)),
            float(data['t_fixed']),
            float(data['t_hold']),
            float(data['t_return']),
            int(data['ir_debounce_ms']),
        )
        cfg.validate()
        return cfg

    def validate(self):
        if not (100 <= self.belt_steps_per_sec <= 10000):
            raise ValueError('belt_steps_per_sec 범위는 100~10000 이어야 합니다.')
        if not (1000 <= self.ir_to_sorter_distance_steps <= 5000000):
            raise ValueError('ir_to_sorter_distance_steps 범위는 1000~5000000 이어야 합니다.')
        for name, val in [('t_fixed', self.t_fixed), ('t_hold', self.t_hold), ('t_return', self.t_return)]:
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
        return HardwareConfig.from_dict(json.loads(CONFIG_PATH.read_text(encoding='utf-8')))
    except Exception as exc:
        print(f"설정 파일 로드 실패. 기본값 사용: {exc}")
        cfg = HardwareConfig.defaults()
        save_hw_config(cfg)
        return cfg


def save_hw_config(cfg: HardwareConfig):
    ensure_config_dir()
    CONFIG_PATH.write_text(json.dumps(asdict(cfg), ensure_ascii=False, indent=2), encoding='utf-8')


def calculate_5th_polynomial_coeffs(target_steps):
    s_f = float(target_steps)
    return 0.0, 0.0, 0.0, 10.0 * s_f, -15.0 * s_f, 6.0 * s_f


def _checksum(packet_type: int, payload: bytes) -> int:
    csum = packet_type & 0xFF
    for b in payload:
        csum ^= b
    return csum & 0xFF


def build_packet(packet_type: int, payload: bytes) -> bytes:
    return bytes([PACKET_START, packet_type, *payload, _checksum(packet_type, payload), PACKET_END])


def build_class_result_packet(object_id: int, classify_flag: int) -> bytes:
    payload = struct.pack('<Ii', int(object_id), int(classify_flag))
    return build_packet(PACKET_TYPE_CLASS_RESULT, payload)


def build_coeff_packet(coeffs: tuple) -> bytes:
    return build_packet(PACKET_TYPE_COEF, struct.pack('<6f', *[float(x) for x in coeffs]))


def build_cfg_packet(cfg: HardwareConfig) -> bytes:
    payload = struct.pack(
        '<4f2I',
        float(cfg.belt_steps_per_sec),
        float(cfg.t_fixed),
        float(cfg.t_hold),
        float(cfg.t_return),
        int(cfg.ir_debounce_ms),
        int(cfg.ir_to_sorter_distance_steps),
    )
    return build_packet(PACKET_TYPE_CFG, payload)


def init_hardware():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"경고: 카메라 {CAMERA_INDEX}를 열 수 없습니다.")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(2.5)
        ser.reset_input_buffer()
    except Exception as e:
        print(f'시리얼 연결 오류: {e}')
        ser = None
    return cap, ser


def classify_image_with_ai(client: genai.Client, frame, prompt_text: str):
    success, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not success:
        return 0, '이미지 인코딩 실패'
    try:
        response = client.models.generate_content(
            model=AI_MODEL_NAME,
            contents=[types.Part.from_bytes(data=encoded_img.tobytes(), mime_type='image/jpeg'), prompt_text],
            config=types.GenerateContentConfig(thinking_config=types.ThinkingConfig(thinking_level='MINIMAL')),
        )
        result_text = response.text.strip()
        match = re.search(r'\[\s*(-?\d+)\s*\]', result_text)
        if match:
            return (1 if int(match.group(1)) == 1 else 0), result_text
        if result_text == '1':
            return 1, result_text
        return 0, result_text
    except Exception as e:
        return 0, f'[오류] {e}'


def send_class_result_to_arduino(ser: serial.Serial, object_id: int, classify_flag: int):
    if ser is None or not ser.is_open:
        return
    with serial_lock:
        ser.write(build_class_result_packet(object_id, classify_flag))


class HardwareController(QThread):
    frame_ready = pyqtSignal(object)
    ai_image_ready = pyqtSignal(object)
    classification_finished = pyqtSignal(int, float, int, tuple, str)
    connection_status = pyqtSignal(bool, bool)
    api_status = pyqtSignal(bool)
    queue_updated = pyqtSignal(int)
    settings_apply_result = pyqtSignal(bool, str, dict)

    def __init__(self, target_steps, client, hw_config: HardwareConfig):
        super().__init__()
        self.target_steps = target_steps
        self.client = client
        self.running = True
        self.cap = None
        self.ser = None
        self._pending_count = 0
        self._count_lock = threading.Lock()
        self._ai_semaphore = threading.Semaphore(8)
        self.hw_config = hw_config
        self.prompt_text = DEFAULT_AI_PROMPT
        self.cached_coeffs = calculate_5th_polynomial_coeffs(target_steps)
        self._config_lock = threading.Lock()
        self._pending_config_dict = None
        self._unclassified_ids = []
        self._id_lock = threading.Lock()
        self._ir_trigger_event = threading.Event()

    def trigger_test_event(self):
        self._ir_trigger_event.set()

    @pyqtSlot(str)
    def update_prompt(self, prompt_text: str):
        prompt_text = (prompt_text or '').strip()
        if prompt_text:
            self.prompt_text = prompt_text

    @pyqtSlot(dict)
    def request_settings_apply(self, cfg_dict: dict):
        with self._config_lock:
            self._pending_config_dict = cfg_dict

    def _send_packet_with_ack(self, packet: bytes, ok_ack: str, err_ack: str, busy_ack: str = '', retries: int = 3, timeout_sec: float = 0.2):
        if self.ser is None or not self.ser.is_open:
            return False, '시리얼 포트가 연결되어 있지 않습니다.'
        for _ in range(retries):
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
                    if line.startswith('IR_DETECTED:'):
                        with self._id_lock:
                            self._unclassified_ids.append(int(line.split(':', 1)[1]))
                        continue
                    if line == ok_ack:
                        return True, ok_ack
                    if busy_ack and line == busy_ack:
                        return False, busy_ack
                    if line == err_ack:
                        break
        return False, 'ACK_TIMEOUT'

    def _sync_arduino_startup(self):
        ok_cfg, _ = self._send_packet_with_ack(build_cfg_packet(self.hw_config), ACK_CFG_OK, ACK_CFG_ERR, ACK_CFG_BUSY, 3, 0.3)
        if not ok_cfg:
            return False
        ok_coef, _ = self._send_packet_with_ack(build_coeff_packet(self.cached_coeffs), ACK_COEF_OK, ACK_COEF_ERR, '', 3, 0.3)
        return ok_coef

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
        ok_cfg, msg_cfg = self._send_packet_with_ack(build_cfg_packet(new_cfg), ACK_CFG_OK, ACK_CFG_ERR, ACK_CFG_BUSY, 3, 0.25)
        if not ok_cfg:
            self.settings_apply_result.emit(False, '소터 동작 중 또는 ACK 실패', {})
            return
        ok_coef, msg_coef = self._send_packet_with_ack(build_coeff_packet(self.cached_coeffs), ACK_COEF_OK, ACK_COEF_ERR, '', 3, 0.25)
        if not ok_coef:
            self.settings_apply_result.emit(False, f'계수 재동기화 실패: {msg_coef}', asdict(new_cfg))
            self.hw_config = new_cfg
            return
        self.hw_config = new_cfg
        self.settings_apply_result.emit(True, '하드웨어 설정 적용 완료', asdict(new_cfg))

    def process_classification_task(self, frame_copy, start_time):
        try:
            with self._ai_semaphore:
                classify_flag, raw_text = classify_image_with_ai(self.client, frame_copy, self.prompt_text) if (self.client and frame_copy is not None) else (0, '0')

            api_delay = time.time() - start_time
            with self._id_lock:
                object_id = self._unclassified_ids.pop(0) if self._unclassified_ids else None

            if object_id is None:
                return

            send_class_result_to_arduino(self.ser, object_id, classify_flag)
            self.classification_finished.emit(classify_flag, api_delay, object_id, self.cached_coeffs, raw_text)
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
            self.client.models.generate_content(model=AI_MODEL_NAME, contents=['ping'])
            self.api_status.emit(True)
        except Exception:
            self.api_status.emit(False)

    def run(self):
        self.cap, self.ser = init_hardware()
        self.connection_status.emit(self.cap is not None and self.cap.isOpened(), self.ser is not None and self.ser.is_open)
        if self.ser and self.ser.is_open:
            self._sync_arduino_startup()
        threading.Thread(target=self._ping_api, daemon=True).start()

        while self.running:
            self._process_pending_settings_if_any()
            ret, frame = self.cap.read() if self.cap and self.cap.isOpened() else (False, None)
            if ret and frame is not None:
                self.frame_ready.emit(frame)

            if self._ir_trigger_event.is_set():
                self._ir_trigger_event.clear()
                with self._id_lock:
                    object_id = (max(self._unclassified_ids) + 1) if self._unclassified_ids else 1000000
                    self._unclassified_ids.append(object_id)
                start_time = time.time()
                frame_to_process = frame.copy() if (ret and frame is not None) else None
                if frame_to_process is not None:
                    self.ai_image_ready.emit(frame_to_process)
                with self._count_lock:
                    self._pending_count += 1
                    count = self._pending_count
                self.queue_updated.emit(count)
                threading.Thread(target=self.process_classification_task, args=(frame_to_process, start_time), daemon=True).start()

            if self.ser and self.ser.in_waiting > 0:
                try:
                    signal = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if signal.startswith('IR_DETECTED:'):
                        object_id = int(signal.split(':', 1)[1])
                        with self._id_lock:
                            self._unclassified_ids.append(object_id)
                        start_time = time.time()
                        frame_to_process = frame.copy() if (ret and frame is not None) else None
                        if frame_to_process is not None:
                            self.ai_image_ready.emit(frame_to_process)
                        with self._count_lock:
                            self._pending_count += 1
                            count = self._pending_count
                        self.queue_updated.emit(count)
                        threading.Thread(target=self.process_classification_task, args=(frame_to_process, start_time), daemon=True).start()
                except Exception:
                    pass

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
    client = genai.Client(vertexai=True, api_key=api_key) if api_key else None

    hw_config = load_hw_config()
    steps_per_degree = (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEP_SETTING) / 360.0
    target_steps = int(SORTER_TARGET_DEGREE * steps_per_degree)

    app = QApplication([])
    dashboard = DashboardApp(initial_prompt=DEFAULT_AI_PROMPT)
    dashboard.set_hardware_info(SORTER_TARGET_DEGREE, hw_config.belt_steps_per_sec, MOTOR_MICROSTEP_SETTING, MOTOR_FULL_STEPS_PER_REV, hw_config.ir_to_sorter_distance_steps, target_steps, SERIAL_PORT, BAUD_RATE, CAMERA_INDEX, AI_MODEL_NAME)
    dashboard.set_runtime_settings(asdict(hw_config))
    dashboard.show()

    hw_thread = HardwareController(target_steps, client, hw_config)
    hw_thread.frame_ready.connect(dashboard.set_live_frame)
    hw_thread.ai_image_ready.connect(dashboard.set_ai_image)
    hw_thread.classification_finished.connect(dashboard.add_classification_result)
    hw_thread.connection_status.connect(dashboard.update_connection_status)
    hw_thread.api_status.connect(dashboard.update_api_status)
    hw_thread.queue_updated.connect(dashboard.update_queue_count)
    dashboard.test_ir_signal.connect(hw_thread.trigger_test_event)
    dashboard.settings_save_requested.connect(hw_thread.request_settings_apply)
    dashboard.prompt_save_requested.connect(hw_thread.update_prompt)

    def handle_settings_result(success: bool, message: str, applied_values: dict):
        nonlocal hw_config
        if success and applied_values:
            try:
                hw_config = HardwareConfig.from_dict(applied_values)
                save_hw_config(hw_config)
                dashboard.set_hardware_info(SORTER_TARGET_DEGREE, hw_config.belt_steps_per_sec, MOTOR_MICROSTEP_SETTING, MOTOR_FULL_STEPS_PER_REV, hw_config.ir_to_sorter_distance_steps, target_steps, SERIAL_PORT, BAUD_RATE, CAMERA_INDEX, AI_MODEL_NAME)
                dashboard.set_runtime_settings(asdict(hw_config))
            except Exception as exc:
                message = f'{message} (로컬 저장 실패: {exc})'
        dashboard.handle_settings_apply_result(success, message, applied_values if success else None)

    hw_thread.settings_apply_result.connect(handle_settings_result)

    hw_thread.start()
    exit_code = app.exec()
    hw_thread.stop()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
