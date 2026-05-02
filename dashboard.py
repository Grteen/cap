import sys
import time

import cv2
import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont, QImage, QPixmap
from PyQt6.QtWidgets import (
    QDialog,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QDoubleSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class HardwareSettingsDialog(QDialog):
    save_requested = pyqtSignal(dict)

    def __init__(self, initial_values: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle('하드웨어 설정')
        self.setModal(True)
        self.resize(420, 320)

        self._pending = False
        self._last_saved = dict(initial_values)

        root = QVBoxLayout(self)
        form = QFormLayout()

        self.spin_belt = QSpinBox()
        self.spin_belt.setRange(100, 10000)
        self.spin_belt.setValue(int(initial_values.get('belt_steps_per_sec', 2000)))
        form.addRow('BELT_STEPS_PER_SEC', self.spin_belt)

        self.spin_t_fixed = QDoubleSpinBox()
        self.spin_t_fixed.setRange(0.1, 5.0)
        self.spin_t_fixed.setDecimals(3)
        self.spin_t_fixed.setSingleStep(0.01)
        self.spin_t_fixed.setValue(float(initial_values.get('t_fixed', 0.4)))
        form.addRow('T_FIXED (sec)', self.spin_t_fixed)

        self.spin_ir_distance = QSpinBox()
        self.spin_ir_distance.setRange(1000, 5000000)
        self.spin_ir_distance.setValue(int(initial_values.get('ir_to_sorter_distance_steps', 100000)))
        form.addRow('IR_TO_SORTER_DISTANCE_STEPS', self.spin_ir_distance)

        self.spin_t_hold = QDoubleSpinBox()
        self.spin_t_hold.setRange(0.1, 5.0)
        self.spin_t_hold.setDecimals(3)
        self.spin_t_hold.setSingleStep(0.01)
        self.spin_t_hold.setValue(float(initial_values.get('t_hold', 0.8)))
        form.addRow('T_HOLD (sec)', self.spin_t_hold)

        self.spin_t_return = QDoubleSpinBox()
        self.spin_t_return.setRange(0.1, 5.0)
        self.spin_t_return.setDecimals(3)
        self.spin_t_return.setSingleStep(0.01)
        self.spin_t_return.setValue(float(initial_values.get('t_return', 0.4)))
        form.addRow('T_RETURN (sec)', self.spin_t_return)

        self.spin_ir_debounce = QSpinBox()
        self.spin_ir_debounce.setRange(100, 5000)
        self.spin_ir_debounce.setValue(int(initial_values.get('ir_debounce_ms', 500)))
        form.addRow('IR_DEBOUNCE_MS', self.spin_ir_debounce)

        self.spin_ai_delay = QDoubleSpinBox()
        self.spin_ai_delay.setRange(0.0, 10.0)
        self.spin_ai_delay.setDecimals(3)
        self.spin_ai_delay.setSingleStep(0.05)
        self.spin_ai_delay.setValue(float(initial_values.get('ai_forced_delay_sec', 0.0)))
        form.addRow('AI_FORCED_DELAY_SEC', self.spin_ai_delay)

        self.lbl_status = QLabel('')
        self.lbl_status.setStyleSheet('color: #BDBDBD;')

        btn_row = QHBoxLayout()
        self.btn_reset = QPushButton('이전값으로 초기화')
        self.btn_save = QPushButton('저장')
        self.btn_close = QPushButton('닫기')

        self.btn_reset.clicked.connect(self._reset_to_saved)
        self.btn_save.clicked.connect(self._on_save)
        self.btn_close.clicked.connect(self.reject)

        btn_row.addWidget(self.btn_reset)
        btn_row.addStretch(1)
        btn_row.addWidget(self.btn_save)
        btn_row.addWidget(self.btn_close)

        root.addLayout(form)
        root.addWidget(self.lbl_status)
        root.addStretch(1)
        root.addLayout(btn_row)

    def _current_values(self) -> dict:
        return {
            'belt_steps_per_sec': int(self.spin_belt.value()),
            'ir_to_sorter_distance_steps': int(self.spin_ir_distance.value()),
            't_fixed': float(self.spin_t_fixed.value()),
            't_hold': float(self.spin_t_hold.value()),
            't_return': float(self.spin_t_return.value()),
            'ir_debounce_ms': int(self.spin_ir_debounce.value()),
            'ai_forced_delay_sec': float(self.spin_ai_delay.value()),
        }

    def _is_dirty(self) -> bool:
        cur = self._current_values()
        return cur != self._last_saved

    def _set_pending(self, pending: bool):
        self._pending = pending
        self.btn_save.setEnabled(not pending)
        self.btn_reset.setEnabled(not pending)
        self.btn_close.setEnabled(not pending)

    def _reset_to_saved(self):
        self.spin_belt.setValue(int(self._last_saved.get('belt_steps_per_sec', 2000)))
        self.spin_ir_distance.setValue(int(self._last_saved.get('ir_to_sorter_distance_steps', 100000)))
        self.spin_t_fixed.setValue(float(self._last_saved.get('t_fixed', 0.4)))
        self.spin_t_hold.setValue(float(self._last_saved.get('t_hold', 0.8)))
        self.spin_t_return.setValue(float(self._last_saved.get('t_return', 0.4)))
        self.spin_ir_debounce.setValue(int(self._last_saved.get('ir_debounce_ms', 500)))
        self.spin_ai_delay.setValue(float(self._last_saved.get('ai_forced_delay_sec', 0.0)))
        self.lbl_status.setText('마지막 저장값으로 되돌렸습니다.')
        self.lbl_status.setStyleSheet('color: #03A9F4;')

    def _on_save(self):
        if self._pending:
            return

        self._set_pending(True)
        self.lbl_status.setText('아두이노 적용 중...')
        self.lbl_status.setStyleSheet('color: #FFC107;')
        self.save_requested.emit(self._current_values())

    def handle_apply_result(self, success: bool, message: str, applied_values: dict | None):
        self._set_pending(False)
        if success:
            if applied_values:
                self._last_saved = dict(applied_values)
            self.lbl_status.setText(message)
            self.lbl_status.setStyleSheet('color: #4CAF50;')
            self.accept()
            return

        self.lbl_status.setText(message)
        self.lbl_status.setStyleSheet('color: #FF5252;')

    def reject(self):
        if self._pending:
            QMessageBox.warning(self, '처리 중', '저장 처리 중에는 닫을 수 없습니다.')
            return

        if self._is_dirty():
            answer = QMessageBox.question(
                self,
                '저장되지 않은 변경사항',
                '저장되지 않은 변경사항이 있습니다. 저장하지 않고 닫으시겠습니까?',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return

        super().reject()


class DashboardApp(QMainWindow):
    test_ir_signal = pyqtSignal()
    settings_save_requested = pyqtSignal(dict)
    prompt_save_requested = pyqtSignal(str)

    def __init__(self, initial_prompt: str):
        super().__init__()
        self.setWindowTitle('AI Conveyor Sorting System Dashboard')
        self.resize(1200, 800)
        self.setStyleSheet('background-color: #1e1e1e; color: #ffffff;')

        self.runtime_settings = {
            'belt_steps_per_sec': 2000,
            't_fixed': 0.4,
            't_hold': 0.8,
            't_return': 0.4,
            'ir_debounce_ms': 500,
            'ai_forced_delay_sec': 0.0,
        }
        self.initial_prompt = initial_prompt
        self._settings_dialog = None

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QGridLayout(self.central_widget)

        self.init_column1()
        self.init_column2()
        self.init_column3()

    def init_column1(self):
        self.lbl_live_feed = QLabel('라이브 스트리밍 대기 중...')
        self.lbl_live_feed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_live_feed.setStyleSheet('background-color: #000000; border: 1px solid #555;')
        self.lbl_live_feed.setMinimumSize(320, 180)

        lbl_prompt_title = QLabel('현재 AI 프롬프트')
        lbl_prompt_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.txt_prompt = QTextEdit()
        self.txt_prompt.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 11pt;')
        self.txt_prompt.setText(self.initial_prompt)
        self.txt_prompt.setMinimumHeight(100)
        self.btn_save_prompt = QPushButton('프롬프트 저장')
        self.btn_save_prompt.setStyleSheet(
            'QPushButton {background-color: #607D8B; color: white; font-weight: bold; padding: 8px; border-radius: 4px;}'
            'QPushButton:pressed {background-color: #455A64; padding-top: 9px; padding-bottom: 7px;}'
        )
        self.btn_save_prompt.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save_prompt.clicked.connect(self._on_save_prompt)

        lbl_ai_img_title = QLabel('AI 전송 이미지 (마지막 캡처)')
        lbl_ai_img_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.lbl_ai_img = QLabel('No Image Sent Yet')
        self.lbl_ai_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ai_img.setStyleSheet('background-color: #222222; border: 1px solid #555;')
        self.lbl_ai_img.setMinimumSize(320, 180)

        lbl_ai_res_title = QLabel('AI 모델 응답 내용 (전체)')
        lbl_ai_res_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.txt_ai_res = QTextEdit()
        self.txt_ai_res.setReadOnly(True)
        self.txt_ai_res.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 11pt;')
        self.txt_ai_res.setPlaceholderText('AI 응답 대기 중...')
        self.txt_ai_res.setMinimumHeight(120)

        vbox = QVBoxLayout()
        vbox.addWidget(self.lbl_live_feed, stretch=3)
        vbox.addWidget(lbl_prompt_title)
        vbox.addWidget(self.txt_prompt, stretch=2)
        vbox.addWidget(self.btn_save_prompt)
        vbox.addWidget(lbl_ai_img_title)
        vbox.addWidget(self.lbl_ai_img, stretch=3)
        vbox.addWidget(lbl_ai_res_title)
        vbox.addWidget(self.txt_ai_res, stretch=2)

        container = QWidget()
        container.setLayout(vbox)
        self.layout.addWidget(container, 0, 0)

    def _on_save_prompt(self):
        prompt_text = self.txt_prompt.toPlainText().strip()
        if not prompt_text:
            return

        self.prompt_save_requested.emit(prompt_text)
        self._show_prompt_saved_feedback()

    def _show_prompt_saved_feedback(self):
        self.btn_save_prompt.setEnabled(False)
        self.btn_save_prompt.setText('저장되었습니다')
        self.btn_save_prompt.setStyleSheet(
            'QPushButton {background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; border-radius: 4px;}'
            'QPushButton:pressed {background-color: #3d8f40; padding-top: 9px; padding-bottom: 7px;}'
        )
        QTimer.singleShot(2000, self._restore_prompt_save_button)

    def _restore_prompt_save_button(self):
        self.btn_save_prompt.setText('프롬프트 저장')
        self.btn_save_prompt.setEnabled(True)
        self.btn_save_prompt.setStyleSheet(
            'QPushButton {background-color: #607D8B; color: white; font-weight: bold; padding: 8px; border-radius: 4px;}'
            'QPushButton:pressed {background-color: #455A64; padding-top: 9px; padding-bottom: 7px;}'
        )

    def init_column2(self):
        vbox = QVBoxLayout()

        self.plot_latency = pg.PlotWidget(title='AI API 지연 시간 (초 - Latency)')
        self.plot_latency.setBackground('#2b2b2b')
        self.plot_latency.showGrid(x=True, y=True, alpha=0.3)
        self.latency_data = []
        self.latency_curve = self.plot_latency.plot(pen=pg.mkPen('#FF5252', width=2))

        lbl_flag_title = QLabel('분류 결과 플래그')
        lbl_flag_title.setFont(QFont('Arial', 10, QFont.Weight.Bold))
        lbl_flag_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_flag = QLabel('-')
        self.lbl_flag.setFont(QFont('Arial', 48, QFont.Weight.Bold))
        self.lbl_flag.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_flag.setStyleSheet('color: #BDBDBD; background-color: #2b2b2b; border: 3px solid #555; border-radius: 8px; padding: 4px;')
        self.lbl_flag.setFixedHeight(72)

        self.lbl_status = QLabel('대기 상태')
        self.lbl_status.setFont(QFont('Arial', 26, QFont.Weight.Bold))
        self.lbl_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_status.setStyleSheet('color: #BDBDBD;')

        self.lbl_object_id = QLabel('Object ID: --')
        self.lbl_object_id.setFont(QFont('Arial', 32, QFont.Weight.Bold))
        self.lbl_object_id.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_object_id.setStyleSheet('color: #4CAF50; border: 2px solid #4CAF50; padding: 10px;')

        self.lbl_sys_time = QLabel('발생 시각: --:--:--')
        self.lbl_sys_time.setFont(QFont('Arial', 12))

        hbox_conn = QHBoxLayout()
        hbox_conn.setSpacing(16)

        self.lbl_cam_led = QLabel('●')
        self.lbl_cam_led.setStyleSheet('color: #F44336; font-size: 22px;')
        hbox_conn.addWidget(self.lbl_cam_led)
        hbox_conn.addWidget(QLabel('카메라'))

        hbox_conn.addSpacing(8)
        self.lbl_ard_led = QLabel('●')
        self.lbl_ard_led.setStyleSheet('color: #F44336; font-size: 22px;')
        hbox_conn.addWidget(self.lbl_ard_led)
        hbox_conn.addWidget(QLabel('아두이노'))

        hbox_conn.addSpacing(8)
        self.lbl_api_led = QLabel('●')
        self.lbl_api_led.setStyleSheet('color: #F44336; font-size: 22px;')
        hbox_conn.addWidget(self.lbl_api_led)
        hbox_conn.addWidget(QLabel('AI API'))
        hbox_conn.addStretch()

        vbox.addWidget(self.plot_latency, stretch=4)
        vbox.addSpacing(6)
        vbox.addWidget(lbl_flag_title)
        vbox.addWidget(self.lbl_flag)
        vbox.addSpacing(6)
        vbox.addWidget(QLabel("<h3 style='color:#FFF;'>분류 진행 상태</h3>"))
        vbox.addWidget(self.lbl_status)
        vbox.addWidget(self.lbl_object_id)
        vbox.addSpacing(20)
        vbox.addWidget(QLabel("<h3 style='color:#FFF;'>시스템 정보 연결 상태</h3>"))
        vbox.addWidget(self.lbl_sys_time)
        vbox.addLayout(hbox_conn)
        vbox.addSpacing(10)

        self.lbl_queue = QLabel('처리 대기열: 0 건')
        self.lbl_queue.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        self.lbl_queue.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_queue.setStyleSheet('color: #4CAF50; padding: 4px;')
        vbox.addWidget(self.lbl_queue)

        self.lbl_ai_forced_delay = QLabel('AI 강제 지연: 0.000 초')
        self.lbl_ai_forced_delay.setFont(QFont('Arial', 12))
        self.lbl_ai_forced_delay.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ai_forced_delay.setStyleSheet('color: #BDBDBD;')
        vbox.addWidget(self.lbl_ai_forced_delay)
        vbox.addSpacing(6)

        self.btn_test = QPushButton('테스트 동작 (IR 센서 시뮬레이션)')
        self.btn_test.setStyleSheet('background-color: #FF9800; color: white; font-weight: bold; padding: 12px; font-size: 15px; border-radius: 5px;')
        self.btn_test.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_test.clicked.connect(self.test_ir_signal.emit)

        self.btn_settings = QPushButton('하드웨어 설정')
        self.btn_settings.setStyleSheet('background-color: #03A9F4; color: white; font-weight: bold; padding: 12px; font-size: 15px; border-radius: 5px;')
        self.btn_settings.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_settings.clicked.connect(self.open_settings_dialog)

        vbox.addWidget(self.btn_test)
        vbox.addWidget(self.btn_settings)
        vbox.addStretch(0)

        self.layout.addLayout(vbox, 0, 1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)

    def init_column3(self):
        vbox = QVBoxLayout()

        self.plot_poly = pg.PlotWidget(title='Rest-to-Rest 5차 다항식 궤적 최적화')
        self.plot_poly.setBackground('#2b2b2b')
        self.plot_poly.showGrid(x=True, y=True, alpha=0.3)
        self.plot_poly.setLabel('left', '목표 스텝 위치 (종단점)')
        self.plot_poly.setLabel('bottom', '정규화된 시간 u (0~1)')
        self.poly_curve = self.plot_poly.plot(pen=pg.mkPen('#03A9F4', width=2))

        self.lbl_coeffs = QLabel('c0: 0 | c1: 0 | c2: 0 | c3: 0 | c4: 0 | c5: 0')
        self.lbl_coeffs.setFont(QFont('Arial', 10))
        self.lbl_coeffs.setStyleSheet('background-color: #333333; padding: 5px;')

        self.table_history = QTableWidget(0, 3)
        self.table_history.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table_history.setHorizontalHeaderLabels(['발생 시간', '결과', '지연(초)'])
        self.table_history.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_history.setStyleSheet('background-color: #2b2b2b; color: white;')

        hw_gauges_widget, self.hw_gauge_labels = self._create_hw_gauges()

        vbox.addWidget(self.plot_poly, stretch=2)
        vbox.addWidget(QLabel('<b>적용된 수학적 상수 (Coeffs):</b>'))
        vbox.addWidget(self.lbl_coeffs)
        vbox.addSpacing(10)
        vbox.addWidget(QLabel('<b>작업 히스토리 (History 로그)</b>'))
        vbox.addWidget(self.table_history, stretch=1)
        vbox.addSpacing(10)
        vbox.addWidget(QLabel('<b>하드웨어 물리적 파라미터</b>'))
        vbox.addWidget(hw_gauges_widget)

        self.layout.addLayout(vbox, 0, 2)

    def _make_gauge_cell(self, title, value, unit, color):
        frame = QWidget()
        frame.setStyleSheet(f'background-color: #252525; border: 2px solid {color}; border-radius: 6px;')
        inner = QVBoxLayout(frame)
        inner.setContentsMargins(6, 4, 6, 4)
        inner.setSpacing(1)

        lbl_title = QLabel(title)
        lbl_title.setFont(QFont('Arial', 8))
        lbl_title.setStyleSheet(f'color: {color}; border: none; background: transparent;')
        lbl_title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        lbl_val = QLabel(f'{value}')
        lbl_val.setFont(QFont('Arial', 13, QFont.Weight.Bold))
        lbl_val.setStyleSheet('color: #F5F5F5; border: none; background: transparent;')
        lbl_val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        lbl_unit = QLabel(unit)
        lbl_unit.setFont(QFont('Arial', 8))
        lbl_unit.setStyleSheet('color: #AAAAAA; border: none; background: transparent;')
        lbl_unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        inner.addWidget(lbl_title)
        inner.addWidget(lbl_val)
        inner.addWidget(lbl_unit)
        return frame, lbl_val

    def _create_hw_gauges(self):
        gauges = [
            ('목표 각도', '--', '°', '#FF9800'),
            ('벨트 속도', '--', 'steps/s', '#03A9F4'),
            ('마이크로스텝', '--', '× 분할', '#9C27B0'),
            ('풀스텝/회전', '--', 'steps/rev', '#4CAF50'),
            ('IR→분류 거리', '--', 'steps', '#FF5252'),
            ('목표 이동 스텝', '--', 'steps', '#FFC107'),
            ('시리얼 포트', '--', '', '#00BCD4'),
            ('보드레이트', '--', 'bps', '#8BC34A'),
            ('카메라 인덱스', '--', 'idx', '#E91E63'),
            ('AI 모델', '--', '', '#607D8B'),
        ]

        container = QWidget()
        grid = QGridLayout(container)
        grid.setSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)

        gauge_labels = {}
        for i, (title, val, unit, color) in enumerate(gauges):
            row, col = divmod(i, 2)
            cell, lbl_val = self._make_gauge_cell(title, val, unit, color)
            grid.addWidget(cell, row, col)
            gauge_labels[title] = lbl_val

        return container, gauge_labels

    def update_time(self):
        self.lbl_sys_time.setText(f"시간: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
            event.accept()
            return
        super().keyPressEvent(event)

    def open_settings_dialog(self):
        if self._settings_dialog is not None and self._settings_dialog.isVisible():
            self._settings_dialog.activateWindow()
            self._settings_dialog.raise_()
            return

        self._settings_dialog = HardwareSettingsDialog(self.runtime_settings, self)
        self._settings_dialog.save_requested.connect(self.settings_save_requested.emit)
        self._settings_dialog.finished.connect(self._on_settings_dialog_closed)
        self._settings_dialog.show()

    def _on_settings_dialog_closed(self, _result):
        self._settings_dialog = None

    def handle_settings_apply_result(self, success: bool, message: str, applied_values: dict | None):
        if self._settings_dialog is not None:
            self._settings_dialog.handle_apply_result(success, message, applied_values)

    def set_runtime_settings(self, settings: dict):
        self.runtime_settings = dict(settings)
        forced_delay = float(self.runtime_settings.get('ai_forced_delay_sec', 0.0))
        self.lbl_ai_forced_delay.setText(f'AI 강제 지연: {forced_delay:.3f} 초')

    @pyqtSlot(object)
    def set_live_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix_h = self.lbl_live_feed.height()
        if pix_h > 0:
            pix = QPixmap.fromImage(qimg).scaledToHeight(pix_h, Qt.TransformationMode.SmoothTransformation)
            self.lbl_live_feed.setPixmap(pix)

    @pyqtSlot(object)
    def set_ai_image(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix_h = self.lbl_ai_img.height()
        if pix_h > 0:
            pix = QPixmap.fromImage(qimg).scaledToHeight(pix_h, Qt.TransformationMode.SmoothTransformation)
            self.lbl_ai_img.setPixmap(pix)

    @pyqtSlot(int, float, int, tuple, str)
    def add_classification_result(self, result_flag, latency, object_id, coeffs, raw_text):
        self.txt_ai_res.setText(raw_text)

        if result_flag == 1:
            self.lbl_flag.setText('1')
            self.lbl_flag.setStyleSheet('color: #1e1e1e; background-color: #FFC107; border: 3px solid #FFC107; border-radius: 10px; padding: 10px;')
            self.lbl_status.setText('동작 분류 완료 (1)')
            self.lbl_status.setStyleSheet('color: #FFC107;')
        else:
            self.lbl_flag.setText('0')
            self.lbl_flag.setStyleSheet('color: #1e1e1e; background-color: #03A9F4; border: 3px solid #03A9F4; border-radius: 10px; padding: 10px;')
            self.lbl_status.setText('분류 패스됨 (0)')
            self.lbl_status.setStyleSheet('color: #03A9F4;')

        self.lbl_object_id.setText(f'Object ID: {object_id}')

        self.latency_data.append(latency)
        if len(self.latency_data) > 30:
            self.latency_data.pop(0)
        self.latency_curve.setData(self.latency_data)

        c0, c1, c2, c3, c4, c5 = coeffs
        u = np.linspace(0, 1, 100)
        s_u = c0 + c1 * u + c2 * (u ** 2) + c3 * (u ** 3) + c4 * (u ** 4) + c5 * (u ** 5)
        self.poly_curve.setData(u, s_u)

        coeff_str = f'c0: {c0:.1f} | c1: {c1:.1f} | c2: {c2:.1f}\nc3: {c3:.1f} | c4: {c4:.1f} | c5: {c5:.1f}'
        self.lbl_coeffs.setText(coeff_str)

        row = self.table_history.rowCount()
        self.table_history.insertRow(row)
        self.table_history.setItem(row, 0, QTableWidgetItem(time.strftime('%H:%M:%S')))
        self.table_history.setItem(row, 1, QTableWidgetItem(str(result_flag)))
        self.table_history.setItem(row, 2, QTableWidgetItem(f'{latency:.3f}'))
        self.table_history.scrollToBottom()

        # 5초 후 상태 라벨 초기화 타이머 시작
        QTimer.singleShot(5000, self.reset_status_labels)

    @pyqtSlot(int)
    def update_queue_count(self, count):
        self.lbl_queue.setText(f'처리 대기열: {count} 건')
        color = '#FF5252' if count >= 3 else '#FF9800' if count > 0 else '#4CAF50'
        self.lbl_queue.setStyleSheet(f'color: {color}; font-size: 14pt; font-weight: bold; padding: 4px;')

    @pyqtSlot(bool, bool)
    def update_connection_status(self, cam_ok, ard_ok):
        color_cam = '#4CAF50' if cam_ok else '#F44336'
        color_ard = '#4CAF50' if ard_ok else '#F44336'
        self.lbl_cam_led.setStyleSheet(f'color: {color_cam}; font-size: 22px;')
        self.lbl_ard_led.setStyleSheet(f'color: {color_ard}; font-size: 22px;')

    def update_api_status(self, api_ok):
        color = '#4CAF50' if api_ok else '#F44336'
        self.lbl_api_led.setStyleSheet(f'color: {color}; font-size: 22px;')

    def reset_status_labels(self):
        """결과 표시 후 라벨들을 초기 대기 상태로 복구"""
        # 현재 처리 중인 대기열이 있다면 초기화하지 않음 (선택 사항)
        try:
            queue_text = self.lbl_queue.text()
            if "0 건" not in queue_text:
                return
        except:
            pass

        self.lbl_flag.setText('-')
        self.lbl_flag.setStyleSheet('color: #BDBDBD; background-color: #2b2b2b; border: 3px solid #555; border-radius: 8px; padding: 4px;')
        self.lbl_flag.setFixedHeight(72)

        self.lbl_status.setText('대기 상태')
        self.lbl_status.setStyleSheet('color: #BDBDBD;')

        self.lbl_object_id.setText('Object ID: --')
        self.lbl_object_id.setStyleSheet('color: #4CAF50; border: 2px solid #4CAF50; padding: 10px;')

    def set_hardware_info(self, target_deg, belt_speed, microstep,
                          full_steps_rev=None, ir_distance=None,
                          target_steps=None, serial_port=None,
                          baud_rate=None, camera_index=None, ai_model=None,
                          ai_forced_delay_sec=None):
        updates = {
            '목표 각도': f'{target_deg}',
            '벨트 속도': f'{belt_speed}',
            '마이크로스텝': f'{microstep}',
            '풀스텝/회전': f'{full_steps_rev}' if full_steps_rev is not None else '--',
            'IR→분류 거리': f'{ir_distance}' if ir_distance is not None else '--',
            '목표 이동 스텝': f'{target_steps}' if target_steps is not None else '--',
            '시리얼 포트': f'{serial_port}' if serial_port is not None else '--',
            '보드레이트': f'{baud_rate}' if baud_rate is not None else '--',
            '카메라 인덱스': f'{camera_index}' if camera_index is not None else '--',
            'AI 모델': f'{ai_model}' if ai_model is not None else '--',
        }
        for key, val in updates.items():
            if key in self.hw_gauge_labels:
                self.hw_gauge_labels[key].setText(val)
