import json
import struct
import sys
import time
from pathlib import Path

import serial
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

FRAME_START = 0xAA
FRAME_END = 0xFF

CMD_JOG = 0x10
CMD_ZERO = 0x11
CMD_GET_POS = 0x12
CMD_SET_DISTANCE = 0x13
CMD_GET_DISTANCE = 0x14

RSP_ACK = 0x90
RSP_ERR = 0x91

RESULT_PATH = Path("calibration/calibration_result.json")


def checksum(frame_type: int, payload: bytes) -> int:
    cs = frame_type ^ (len(payload) & 0xFF)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def build_frame(frame_type: int, payload: bytes = b"") -> bytes:
    length = len(payload) & 0xFF
    cs = checksum(frame_type, payload)
    return bytes([FRAME_START, frame_type, length, *payload, cs, FRAME_END])


class FrameParser:
    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        out: list[tuple[int, bytes]] = []

        while True:
            if len(self._buf) < 5:
                break

            try:
                start_idx = self._buf.index(FRAME_START)
            except ValueError:
                self._buf.clear()
                break

            if start_idx > 0:
                del self._buf[:start_idx]

            if len(self._buf) < 5:
                break

            frame_type = self._buf[1]
            length = self._buf[2]
            frame_len = 1 + 1 + 1 + length + 1 + 1

            if len(self._buf) < frame_len:
                break

            frame = self._buf[:frame_len]
            del self._buf[:frame_len]

            if frame[-1] != FRAME_END:
                continue

            payload = bytes(frame[3 : 3 + length])
            recv_cs = frame[-2]
            if recv_cs != checksum(frame_type, payload):
                continue

            out.append((frame_type, payload))

        return out


class CalibrationDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Calibration Dashboard")
        self.resize(760, 560)

        self.ser: serial.Serial | None = None
        self.parser = FrameParser()
        self.current_pos = 0
        self.saved_distance = 0

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        conn_box = QGroupBox("시리얼 연결")
        conn_grid = QGridLayout(conn_box)

        self.port_edit = QLineEdit("COM3")
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(9600, 2000000)
        self.baud_spin.setValue(115200)

        self.btn_connect = QPushButton("연결")
        self.btn_disconnect = QPushButton("해제")
        self.btn_disconnect.setEnabled(False)

        conn_grid.addWidget(QLabel("포트"), 0, 0)
        conn_grid.addWidget(self.port_edit, 0, 1)
        conn_grid.addWidget(QLabel("보드레이트"), 0, 2)
        conn_grid.addWidget(self.baud_spin, 0, 3)
        conn_grid.addWidget(self.btn_connect, 0, 4)
        conn_grid.addWidget(self.btn_disconnect, 0, 5)

        state_box = QGroupBox("현재 상태")
        state_grid = QGridLayout(state_box)
        self.lbl_connected = QLabel("연결 상태: OFF")
        self.lbl_connected.setStyleSheet("color:#ff5252;font-weight:bold;")
        self.lbl_pos = QLabel("현재 누적 스텝: 0")
        self.lbl_distance = QLabel("저장된 거리 스텝: 0")
        state_grid.addWidget(self.lbl_connected, 0, 0, 1, 2)
        state_grid.addWidget(self.lbl_pos, 1, 0)
        state_grid.addWidget(self.lbl_distance, 1, 1)

        jog_box = QGroupBox("벨트 JOG 제어")
        jog_layout = QVBoxLayout(jog_box)

        row1 = QHBoxLayout()
        row2 = QHBoxLayout()
        self.btn_m100 = QPushButton("-100")
        self.btn_m10 = QPushButton("-10")
        self.btn_m1 = QPushButton("-1")
        self.btn_p1 = QPushButton("+1")
        self.btn_p10 = QPushButton("+10")
        self.btn_p100 = QPushButton("+100")

        for btn in [self.btn_m100, self.btn_m10, self.btn_m1]:
            btn.setMinimumHeight(42)
            row1.addWidget(btn)
        for btn in [self.btn_p1, self.btn_p10, self.btn_p100]:
            btn.setMinimumHeight(42)
            row2.addWidget(btn)

        action_row = QHBoxLayout()
        self.btn_zero = QPushButton("ZERO (현재=0)")
        self.btn_set_distance = QPushButton("현재값을 거리로 저장")
        self.btn_refresh = QPushButton("값 새로고침")
        action_row.addWidget(self.btn_zero)
        action_row.addWidget(self.btn_set_distance)
        action_row.addWidget(self.btn_refresh)

        jog_layout.addLayout(row1)
        jog_layout.addLayout(row2)
        jog_layout.addLayout(action_row)

        log_box = QGroupBox("로그")
        log_layout = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)

        layout.addWidget(conn_box)
        layout.addWidget(state_box)
        layout.addWidget(jog_box)
        layout.addWidget(log_box, stretch=1)

        self.btn_connect.clicked.connect(self.connect_serial)
        self.btn_disconnect.clicked.connect(self.disconnect_serial)

        self.btn_m100.clicked.connect(lambda: self.send_jog(-100))
        self.btn_m10.clicked.connect(lambda: self.send_jog(-10))
        self.btn_m1.clicked.connect(lambda: self.send_jog(-1))
        self.btn_p1.clicked.connect(lambda: self.send_jog(1))
        self.btn_p10.clicked.connect(lambda: self.send_jog(10))
        self.btn_p100.clicked.connect(lambda: self.send_jog(100))
        self.btn_zero.clicked.connect(self.send_zero)
        self.btn_set_distance.clicked.connect(self.send_set_distance)
        self.btn_refresh.clicked.connect(self.send_refresh)

        self.set_controls_enabled(False)
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_serial)
        self.poll_timer.start(20)

    def log(self, text: str):
        stamp = time.strftime("%H:%M:%S")
        self.log_view.append(f"[{stamp}] {text}")

    def set_controls_enabled(self, enabled: bool):
        for w in [
            self.btn_m100,
            self.btn_m10,
            self.btn_m1,
            self.btn_p1,
            self.btn_p10,
            self.btn_p100,
            self.btn_zero,
            self.btn_set_distance,
            self.btn_refresh,
        ]:
            w.setEnabled(enabled)

    def connect_serial(self):
        if self.ser and self.ser.is_open:
            return
        port = self.port_edit.text().strip()
        baud = int(self.baud_spin.value())
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
            time.sleep(0.05)
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.set_controls_enabled(True)
            self.lbl_connected.setText("연결 상태: ON")
            self.lbl_connected.setStyleSheet("color:#4caf50;font-weight:bold;")
            self.log(f"시리얼 연결 성공 ({port}, {baud})")
            self.send_refresh()
        except Exception as exc:
            self.ser = None
            QMessageBox.critical(self, "연결 실패", str(exc))

    def disconnect_serial(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.set_controls_enabled(False)
        self.lbl_connected.setText("연결 상태: OFF")
        self.lbl_connected.setStyleSheet("color:#ff5252;font-weight:bold;")
        self.log("시리얼 연결 해제")

    def send_frame(self, frame_type: int, payload: bytes = b""):
        if self.ser is None or not self.ser.is_open:
            self.log("시리얼 미연결 상태입니다.")
            return
        try:
            self.ser.write(build_frame(frame_type, payload))
        except Exception as exc:
            self.log(f"전송 오류: {exc}")

    def send_jog(self, delta_steps: int):
        payload = struct.pack("<i", int(delta_steps))
        self.send_frame(CMD_JOG, payload)
        self.log(f"JOG 요청: {delta_steps:+d} step")

    def send_zero(self):
        self.send_frame(CMD_ZERO)
        self.log("ZERO 요청")

    def send_set_distance(self):
        self.send_frame(CMD_SET_DISTANCE)
        self.log("SET_DISTANCE 요청")

    def send_refresh(self):
        self.send_frame(CMD_GET_POS)
        self.send_frame(CMD_GET_DISTANCE)
        self.log("GET_POS/GET_DISTANCE 요청")

    def handle_ack(self, payload: bytes):
        if len(payload) != 9:
            self.log("ACK payload 길이 오류")
            return
        cmd_type = payload[0]
        current_pos = struct.unpack("<i", payload[1:5])[0]
        saved_distance = struct.unpack("<i", payload[5:9])[0]

        self.current_pos = current_pos
        self.saved_distance = saved_distance
        self.lbl_pos.setText(f"현재 누적 스텝: {self.current_pos}")
        self.lbl_distance.setText(f"저장된 거리 스텝: {self.saved_distance}")
        self.log(
            f"ACK cmd=0x{cmd_type:02X}, pos={self.current_pos}, distance={self.saved_distance}"
        )

        if cmd_type == CMD_SET_DISTANCE:
            self.persist_result()

    def handle_err(self, payload: bytes):
        if len(payload) != 2:
            self.log("ERR payload 길이 오류")
            return
        cmd_type = payload[0]
        err_code = payload[1]
        self.log(f"ERR cmd=0x{cmd_type:02X}, code={err_code}")

    def poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            data = self.ser.read(n)
            for frame_type, payload in self.parser.feed(data):
                if frame_type == RSP_ACK:
                    self.handle_ack(payload)
                elif frame_type == RSP_ERR:
                    self.handle_err(payload)
                else:
                    self.log(f"알 수 없는 응답 type=0x{frame_type:02X}")
        except Exception as exc:
            self.log(f"수신 오류: {exc}")

    def persist_result(self):
        RESULT_PATH.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "saved_distance_steps": int(self.saved_distance),
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        RESULT_PATH.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        self.log(f"결과 저장 완료: {RESULT_PATH}")

    def closeEvent(self, event):
        self.disconnect_serial()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = CalibrationDashboard()
    window.show()
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()

