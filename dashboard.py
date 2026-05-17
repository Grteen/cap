import os
import sys
import time

import cv2
import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QImage, QPixmap
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
    QSizePolicy,
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class PlainTextEdit(QTextEdit):
    def insertFromMimeData(self, source):
        if source.hasText():
            self.insertPlainText(source.text())
        else:
            super().insertFromMimeData(source)


class AspectRatioWidget(QWidget):
    def __init__(self, widget, ratio_w=16, ratio_h=9, parent=None):
        super().__init__(parent)
        self.ratio_w = ratio_w
        self.ratio_h = ratio_h
        self.ratio = ratio_w / ratio_h
        self.widget = widget
        self.widget.setParent(self)
        
        # Enforce height-for-width aspect ratio scaling
        policy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        policy.setHeightForWidth(True)
        self.setSizePolicy(policy)
        
    def hasHeightForWidth(self):
        return True
        
    def heightForWidth(self, width):
        return int(width * self.ratio_h / self.ratio_w)
        
    def resizeEvent(self, event):
        super().resizeEvent(event)
        w = self.width()
        h = self.height()
        if h == 0 or w == 0:
            return
            
        if w / h > self.ratio:
            new_h = h
            new_w = int(h * self.ratio)
        else:
            new_w = w
            new_h = int(w / self.ratio)
            
        self.widget.setGeometry(
            (w - new_w) // 2,
            (h - new_h) // 2,
            new_w,
            new_h
        )


class PresetEditorDialog(QDialog):
    def __init__(self, preset_num, filepath, parent=None):
        super().__init__(parent)
        self.preset_num = preset_num
        self.filepath = filepath
        self.parent_app = parent
        
        self.setWindowTitle(f'프리셋 {preset_num} 관리')
        self.resize(500, 350)
        self.setStyleSheet('background-color: #1e1e1e; color: #ffffff;')
        
        layout = QVBoxLayout(self)
        
        lbl_title = QLabel(f'✨ 프리셋 {preset_num} 프롬프트 편집 및 관리')
        lbl_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        lbl_title.setStyleSheet('color: #FF9800; padding-bottom: 5px;')
        layout.addWidget(lbl_title)
        
        self.txt_preset = PlainTextEdit()
        self.txt_preset.setStyleSheet('background-color: #2b2b2b; color: #ffffff; border: 1px solid #555; font-size: 11pt;')
        self.txt_preset.setMinimumHeight(180)
        
        saved_text = ""
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, "r", encoding="utf-8") as f:
                    saved_text = f.read()
            except Exception as e:
                saved_text = f"에러: {e}"
        self.txt_preset.setText(saved_text)
        layout.addWidget(self.txt_preset)
        
        hbox = QHBoxLayout()
        
        self.btn_save = QPushButton('💾 저장')
        self.btn_save.setStyleSheet('background-color: #2e7d32; color: white; font-weight: bold; padding: 8px; border-radius: 4px;')
        self.btn_save.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save.clicked.connect(self._on_save)
        hbox.addWidget(self.btn_save)
        
        self.btn_apply = QPushButton('🔌 대시보드에 적용')
        self.btn_apply.setStyleSheet('background-color: #1976d2; color: white; font-weight: bold; padding: 8px; border-radius: 4px;')
        self.btn_apply.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_apply.clicked.connect(self._on_apply)
        hbox.addWidget(self.btn_apply)
        
        self.btn_close = QPushButton('닫기')
        self.btn_close.setStyleSheet('background-color: #555; color: white; padding: 8px; border-radius: 4px;')
        self.btn_close.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_close.clicked.connect(self.reject)
        hbox.addWidget(self.btn_close)
        
        layout.addLayout(hbox)
        
    def _on_save(self):
        preset_text = self.txt_preset.toPlainText().strip()
        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                f.write(preset_text)
            self.btn_save.setEnabled(False)
            self.btn_save.setText('Saved!')
            QTimer.singleShot(1500, lambda: (self.btn_save.setEnabled(True), self.btn_save.setText('💾 저장')))
        except Exception as e:
            QMessageBox.critical(self, '오류', f'프리셋 {self.preset_num} 저장 실패: {e}')
            
    def _on_apply(self):
        preset_text = self.txt_preset.toPlainText()
        if self.parent_app:
            self.parent_app.txt_prompt.setText(preset_text)
            # Instantly save and apply to the running AI system
            self.parent_app._on_save_prompt()
        self.accept()


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
            't_fixed': float(self.spin_t_fixed.value()),
            't_hold': float(self.spin_t_hold.value()),
            't_return': float(self.spin_t_return.value()),
            'ir_debounce_ms': int(self.spin_ir_debounce.value()),
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
        self.spin_belt.setValue(int(self._last_saved['belt_steps_per_sec']))
        self.spin_t_fixed.setValue(float(self._last_saved['t_fixed']))
        self.spin_t_hold.setValue(float(self._last_saved['t_hold']))
        self.spin_t_return.setValue(float(self._last_saved['t_return']))
        self.spin_ir_debounce.setValue(int(self._last_saved['ir_debounce_ms']))
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


class EventDetailDialog(QDialog):
    def __init__(self, event_id, details, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f'작업 상세 정보 (ID: {event_id})')
        self.setModal(True)
        self.resize(750, 500)
        self.setStyleSheet('background-color: #1a1a1a; color: #ffffff;')

        layout = QVBoxLayout(self)

        lbl_header = QLabel(f'🔍 작업 상세 기록 [ID: {event_id}]')
        lbl_header.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        lbl_header.setStyleSheet('color: #03A9F4; padding-bottom: 8px;')
        layout.addWidget(lbl_header)

        hbox = QHBoxLayout()

        vbox_img = QVBoxLayout()
        vbox_img.addWidget(QLabel('<b>전송된 캡처 이미지</b>'))
        lbl_img = QLabel('이미지 없음')
        lbl_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_img.setStyleSheet('background-color: #000000; border: 1px solid #444; min-width: 320px; min-height: 180px; border-radius: 4px;')

        frame_img = details.get("image")
        if frame_img is not None:
            rgb_image = cv2.cvtColor(frame_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pix = QPixmap.fromImage(qimg).scaled(320, 180, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            lbl_img.setPixmap(pix)
        vbox_img.addWidget(lbl_img)
        vbox_img.addStretch(1)
        hbox.addLayout(vbox_img, stretch=1)

        vbox_res = QVBoxLayout()
        vbox_res.addWidget(QLabel('<b>Gemini AI 응답 전문</b>'))
        txt_res = QTextEdit()
        txt_res.setReadOnly(True)
        txt_res.setStyleSheet(
            'background-color: #2b2b2b; '
            'color: #ffffff; '
            'font-family: sans-serif; '
            'font-size: 10pt; '
            'border: 1px solid #444; '
            'border-radius: 4px; '
            'padding: 8px;'
        )
        txt_res.setText(details.get("response", "기록 없음"))
        vbox_res.addWidget(txt_res)
        hbox.addLayout(vbox_res, stretch=1)

        layout.addLayout(hbox)

        btn_close = QPushButton('닫기')
        btn_close.setStyleSheet('background-color: #3e3e3e; color: white; font-weight: bold; padding: 8px; border-radius: 4px;')
        btn_close.setCursor(Qt.CursorShape.PointingHandCursor)
        btn_close.clicked.connect(self.accept)
        layout.addWidget(btn_close)


class DashboardApp(QMainWindow):
    test_ir_signal = pyqtSignal()
    settings_save_requested = pyqtSignal(dict)
    prompt_save_requested = pyqtSignal(str)
    virtual_delay_changed = pyqtSignal(float)

    def __init__(self, initial_prompt: str, run_folder: str = ""):
        super().__init__()
        self.setWindowTitle('AI Conveyor Sorting System Dashboard')
        self.resize(1200, 800)
        self.setStyleSheet('''
            QMainWindow {
                background-color: #1e1e1e;
            }
            QLabel {
                color: #ffffff;
            }
            QScrollBar:vertical {
                border: none;
                background: #14171d;
                width: 10px;
                margin: 0px 0 0px 0;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #3d4454;
                min-height: 20px;
                border-radius: 5px;
            }
            QScrollBar::handle:vertical:hover {
                background: #03A9F4;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                border: none;
                background: none;
                height: 0px;
            }
            QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical {
                border: none;
                background: none;
                height: 0px;
            }
            QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
                background: none;
            }

            QScrollBar:horizontal {
                border: none;
                background: #14171d;
                height: 10px;
                margin: 0px 0 0px 0;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal {
                background: #3d4454;
                min-width: 20px;
                border-radius: 5px;
            }
            QScrollBar::handle:horizontal:hover {
                background: #03A9F4;
            }
            QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
                border: none;
                background: none;
                width: 0px;
            }
            QScrollBar::up-arrow:horizontal, QScrollBar::down-arrow:horizontal {
                border: none;
                background: none;
                width: 0px;
            }
            QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {
                background: none;
            }
        ''')

        self.runtime_settings = {
            'belt_steps_per_sec': 2000,
            't_fixed': 0.4,
            't_hold': 0.8,
            't_return': 0.4,
            'ir_debounce_ms': 500,
        }
        self.virtual_delay_sec = 0.0
        self.initial_prompt = initial_prompt
        self.event_row_map = {}
        self.run_folder = run_folder
        self.last_ai_frame = None
        self._settings_dialog = None

        # Setup Config Paths
        self.config_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config")
        os.makedirs(self.config_dir, exist_ok=True)
        self.prompt_file = os.path.join(self.config_dir, "prompt.txt")
        self.preset1_file = os.path.join(self.config_dir, "preset1.txt")
        self.preset2_file = os.path.join(self.config_dir, "preset2.txt")

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QGridLayout(self.central_widget)

        self.init_column1()
        self.init_column2()
        self.init_column3()

    def init_column1(self):
        lbl_prompt_title = QLabel('프롬프트')
        lbl_prompt_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.txt_prompt = PlainTextEdit()
        self.txt_prompt.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 11pt;')
        self.txt_prompt.setText(self.initial_prompt)
        self.txt_prompt.setMinimumHeight(100)

        hbox_buttons = QHBoxLayout()

        self.btn_save_prompt = QPushButton('💾')
        self.btn_save_prompt.setStyleSheet(
            'QPushButton {background-color: #607D8B; color: white; font-weight: bold; padding: 8px; border-radius: 4px;}'
            'QPushButton:pressed {background-color: #455A64; padding-top: 9px; padding-bottom: 7px;}'
        )
        self.btn_save_prompt.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save_prompt.clicked.connect(self._on_save_prompt)
        hbox_buttons.addWidget(self.btn_save_prompt, stretch=2)

        self.btn_load_p1 = QPushButton('1')
        self.btn_load_p1.setStyleSheet(
            'QPushButton {background-color: #2E7D32; color: white; font-weight: bold; padding: 8px; border-radius: 4px; font-size: 11px;}'
            'QPushButton:pressed {background-color: #1B5E20;}'
        )
        self.btn_load_p1.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_load_p1.clicked.connect(self._on_open_preset1_dialog)
        hbox_buttons.addWidget(self.btn_load_p1, stretch=1)

        self.btn_load_p2 = QPushButton('2')
        self.btn_load_p2.setStyleSheet(
            'QPushButton {background-color: #EF6C00; color: white; font-weight: bold; padding: 8px; border-radius: 4px; font-size: 11px;}'
            'QPushButton:pressed {background-color: #E65100;}'
        )
        self.btn_load_p2.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_load_p2.clicked.connect(self._on_open_preset2_dialog)
        hbox_buttons.addWidget(self.btn_load_p2, stretch=1)

        self.btn_clear_prompt = QPushButton('🗑️')
        self.btn_clear_prompt.setStyleSheet(
            'QPushButton {background-color: #C62828; color: white; font-weight: bold; padding: 8px; border-radius: 4px; font-size: 11px;}'
            'QPushButton:pressed {background-color: #B71C1C;}'
        )
        self.btn_clear_prompt.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_clear_prompt.clicked.connect(self._on_clear_prompt)
        hbox_buttons.addWidget(self.btn_clear_prompt, stretch=1)

        lbl_ai_img_title = QLabel('전송 이미지')
        lbl_ai_img_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.lbl_ai_img = QLabel('⏳')
        self.lbl_ai_img.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ai_img.setStyleSheet('background-color: #222222; border: 1px solid #555; font-size: 14pt;')
        
        # Wrap in AspectRatioWidget to force strict 16:9 ratio
        self.ai_img_container = AspectRatioWidget(self.lbl_ai_img, 16, 9)
        self.ai_img_container.setMinimumSize(320, 180)

        lbl_ai_res_title = QLabel('응답 내용')
        lbl_ai_res_title.setFont(QFont('Arial', 12, QFont.Weight.Bold))
        self.txt_ai_res = QLabel('⏳')
        self.txt_ai_res.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_ai_res.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 14pt;')
        self.txt_ai_res.setMinimumHeight(120)
        self.txt_ai_res.setWordWrap(True)

        vbox = QVBoxLayout()
        vbox.addWidget(lbl_prompt_title)
        vbox.addWidget(self.txt_prompt, stretch=2)
        vbox.addLayout(hbox_buttons)
        vbox.addWidget(lbl_ai_img_title)
        # Set stretch to 0, since its height is strictly determined by its width
        vbox.addWidget(self.ai_img_container, stretch=0)
        vbox.addWidget(lbl_ai_res_title)
        vbox.addWidget(self.txt_ai_res, stretch=2)

        container = QWidget()
        container.setLayout(vbox)
        self.layout.addWidget(container, 0, 0)

    def _on_save_prompt(self):
        prompt_text = self.txt_prompt.toPlainText().strip()
        if not prompt_text:
            return

        try:
            with open(self.prompt_file, "w", encoding="utf-8") as f:
                f.write(prompt_text)
        except Exception as e:
            print(f"프롬프트 파일 저장 실패: {e}")

        self.prompt_save_requested.emit(prompt_text)
        self._show_prompt_saved_feedback()

    def _on_open_preset1_dialog(self):
        dialog = PresetEditorDialog(1, self.preset1_file, self)
        dialog.exec()

    def _on_open_preset2_dialog(self):
        dialog = PresetEditorDialog(2, self.preset2_file, self)
        dialog.exec()

    def _on_clear_prompt(self):
        self.txt_prompt.clear()

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

        self.lbl_live_feed = QLabel('라이브 스트리밍 대기 중...')
        self.lbl_live_feed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_live_feed.setStyleSheet('background-color: #000000; border: 1px solid #555;')
        
        # Wrap in AspectRatioWidget to force strict 16:9 ratio
        self.live_feed_container = AspectRatioWidget(self.lbl_live_feed, 16, 9)
        self.live_feed_container.setMinimumSize(320, 180)
        # Set stretch to 0, since its height is strictly determined by its width
        vbox.addWidget(self.live_feed_container, stretch=0)
        vbox.addSpacing(6)

        self.latency_data = []

        # Nixie-style Classic Digital Clock (Green Glow)
        self.lbl_sys_time = QLabel('--:--:--')
        self.lbl_sys_time.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_sys_time.setStyleSheet(
            'background-color: #0d0f12; '
            'color: #00FF66; '  # Glowing Digital Green
            'border: 2px solid #23272a; '
            'border-radius: 6px; '
            'padding: 8px 16px; '
            'font-family: "Consolas", "Courier New", monospace; '
            'font-size: 20px; '
            'font-weight: bold;'
        )

        hbox_conn = QHBoxLayout()
        hbox_conn.setSpacing(12)
        hbox_conn.addStretch(1)

        # CAMERA LED (Default: Red Glossy Sphere)
        self.lbl_cam_led = QLabel()
        self.lbl_cam_led.setFixedSize(14, 14)
        self.lbl_cam_led.setStyleSheet(
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
            'border: 2px solid #111111; border-radius: 7px;'
        )
        lbl_cam_text = QLabel('CAMERA')
        lbl_cam_text.setFont(QFont('Segoe UI', 9, QFont.Weight.Bold))
        lbl_cam_text.setStyleSheet('color: #CCCCCC; border: none; background: transparent;')
        hbox_conn.addWidget(self.lbl_cam_led, alignment=Qt.AlignmentFlag.AlignVCenter)
        hbox_conn.addWidget(lbl_cam_text, alignment=Qt.AlignmentFlag.AlignVCenter)

        hbox_conn.addSpacing(12)

        # ARDUINO LED (Default: Red Glossy Sphere)
        self.lbl_ard_led = QLabel()
        self.lbl_ard_led.setFixedSize(14, 14)
        self.lbl_ard_led.setStyleSheet(
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
            'border: 2px solid #111111; border-radius: 7px;'
        )
        lbl_ard_text = QLabel('ARDUINO')
        lbl_ard_text.setFont(QFont('Segoe UI', 9, QFont.Weight.Bold))
        lbl_ard_text.setStyleSheet('color: #CCCCCC; border: none; background: transparent;')
        hbox_conn.addWidget(self.lbl_ard_led, alignment=Qt.AlignmentFlag.AlignVCenter)
        hbox_conn.addWidget(lbl_ard_text, alignment=Qt.AlignmentFlag.AlignVCenter)

        hbox_conn.addSpacing(12)

        # AI API LED (Default: Red Glossy Sphere)
        self.lbl_api_led = QLabel()
        self.lbl_api_led.setFixedSize(14, 14)
        self.lbl_api_led.setStyleSheet(
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
            'border: 2px solid #111111; border-radius: 7px;'
        )
        lbl_api_text = QLabel('AI API')
        lbl_api_text.setFont(QFont('Segoe UI', 9, QFont.Weight.Bold))
        lbl_api_text.setStyleSheet('color: #CCCCCC; border: none; background: transparent;')
        hbox_conn.addWidget(self.lbl_api_led, alignment=Qt.AlignmentFlag.AlignVCenter)
        hbox_conn.addWidget(lbl_api_text, alignment=Qt.AlignmentFlag.AlignVCenter)

        hbox_conn.addStretch(1)

        # Retro Control Panel Bezel Box
        panel_widget = QWidget()
        panel_widget.setStyleSheet(
            'background-color: #15181c; '
            'border: 2px solid #282c34; '
            'border-radius: 8px;'
        )
        panel_layout = QVBoxLayout(panel_widget)
        panel_layout.setContentsMargins(12, 12, 12, 12)
        panel_layout.setSpacing(12)
        panel_layout.addWidget(self.lbl_sys_time)
        panel_layout.addLayout(hbox_conn)

        # Let the other elements expand beautifully
        vbox.addWidget(panel_widget)
        vbox.addSpacing(12)



        # Virtual API Delay Simulation Panel
        delay_panel = QWidget()
        delay_panel.setStyleSheet(
            'background-color: #15181c; '
            'border: 2px solid #282c34; '
            'border-radius: 8px;'
        )
        delay_layout = QVBoxLayout(delay_panel)
        delay_layout.setContentsMargins(12, 10, 12, 10)
        delay_layout.setSpacing(6)

        # Title
        lbl_delay_title = QLabel('가상 API 지연')
        lbl_delay_title.setFont(QFont('Segoe UI', 10, QFont.Weight.Normal))
        lbl_delay_title.setStyleSheet('color: #ffffff; border: none; background: transparent;')
        delay_layout.addWidget(lbl_delay_title)

        # Slider + Reset button layout
        slider_hbox = QHBoxLayout()
        slider_hbox.setSpacing(8)

        # Widescreen Slider Control
        self.slider_delay = QSlider(Qt.Orientation.Horizontal)
        self.slider_delay.setRange(0, 30)
        self.slider_delay.setValue(int(self.virtual_delay_sec))
        self.slider_delay.setStyleSheet('''
            QSlider::groove:horizontal {
                border: 1px solid #444;
                height: 6px;
                background: #252830;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #03A9F4;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #ffffff;
                border: 1px solid #888;
                width: 14px;
                margin-top: -4px;
                margin-bottom: -4px;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #03A9F4;
                border-color: #03A9F4;
            }
        ''')
        self.slider_delay.setCursor(Qt.CursorShape.PointingHandCursor)
        self.slider_delay.valueChanged.connect(self._on_virtual_delay_slider_changed)
        slider_hbox.addWidget(self.slider_delay, stretch=4)

        # 0s Quick Reset Button
        self.btn_reset_delay = QPushButton('🔃')
        self.btn_reset_delay.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_reset_delay.setStyleSheet('''
            QPushButton {
                background-color: #2E7D32;
                color: white;
                font-weight: bold;
                padding: 4px 8px;
                border-radius: 4px;
                font-size: 11px;
                border: none;
            }
            QPushButton:hover {
                background-color: #388E3C;
            }
            QPushButton:pressed {
                background-color: #1B5E20;
            }
        ''')
        self.btn_reset_delay.clicked.connect(self._on_reset_delay_clicked)
        slider_hbox.addWidget(self.btn_reset_delay, stretch=1)
        delay_layout.addLayout(slider_hbox)

        # Status output and Preset Buttons side-by-side
        status_hbox = QHBoxLayout()
        
        self.lbl_virtual_delay = QLabel('적용 지연: +0초')
        self.lbl_virtual_delay.setFont(QFont('Segoe UI', 10, QFont.Weight.Normal))
        self.lbl_virtual_delay.setStyleSheet('color: #ffffff; border: none; background: transparent;')
        status_hbox.addWidget(self.lbl_virtual_delay)
        
        status_hbox.addStretch(1)
        
        # Preset buttons for 5s, 10s, 15s
        btn_5s = QPushButton('5초')
        btn_10s = QPushButton('10초')
        btn_15s = QPushButton('15초')
        
        preset_qss = '''
            QPushButton {
                background-color: #2b303c;
                color: #ffffff;
                border: 1px solid #3d4454;
                border-radius: 5px;
                padding: 6px 12px;
                font-size: 11px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3d4454;
            }
            QPushButton:pressed {
                background-color: #1e222a;
            }
        '''
        
        for btn in [btn_5s, btn_10s, btn_15s]:
            btn.setCursor(Qt.CursorShape.PointingHandCursor)
            btn.setStyleSheet(preset_qss)
            status_hbox.addWidget(btn)
            
        btn_5s.clicked.connect(lambda: self.slider_delay.setValue(5))
        btn_10s.clicked.connect(lambda: self.slider_delay.setValue(10))
        btn_15s.clicked.connect(lambda: self.slider_delay.setValue(15))
        
        delay_layout.addLayout(status_hbox)

        vbox.addWidget(delay_panel)
        vbox.addSpacing(12)

        # Modern Sensor Simulation Panel
        trigger_panel = QWidget()
        trigger_panel.setStyleSheet('''
            QWidget {
                background-color: #15181c;
                border: 2px solid #282c34;
                border-radius: 10px;
            }
        ''')
        trigger_layout = QVBoxLayout(trigger_panel)
        trigger_layout.setContentsMargins(14, 14, 14, 14)
        trigger_layout.setSpacing(10)

        # Header: Modern clean title indicating simulation
        lbl_trigger_header = QLabel('IR 센서 시뮬레이션')
        lbl_trigger_header.setFont(QFont('Segoe UI', 10, QFont.Weight.Normal))
        lbl_trigger_header.setStyleSheet('color: #ECEFF1; border: none; background: transparent;')
        lbl_trigger_header.setAlignment(Qt.AlignmentFlag.AlignCenter)
        trigger_layout.addWidget(lbl_trigger_header)

        # Center layout to hold the modern circular tactile button
        btn_center_hbox = QHBoxLayout()
        btn_center_hbox.addStretch(1)

        self.btn_test = QPushButton('')
        self.btn_test.setFixedSize(68, 68)
        self.btn_test.setCursor(Qt.CursorShape.PointingHandCursor)
        
        # Modern tactile push-button style (clean slate with subtle blue LED indicator ring)
        self.btn_test.setStyleSheet('''
            QPushButton {
                /* Matte slate button core with a precise grey boundary */
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5, 
                            stop:0 #2c313d, stop:0.85 #212530, stop:1.0 #171a22);
                border: 3px solid #3d4454;
                border-radius: 34px; /* Half of 68 */
            }
            QPushButton:hover {
                /* Active state: Modern soft blue light ring triggers */
                border-color: #03A9F4;
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5, 
                            stop:0 #323846, stop:0.85 #252a36, stop:1.0 #1a1e27);
            }
            QPushButton:pressed {
                /* Precise micro-travel mechanical depression */
                background: #171a22;
                border-color: #0288D1;
                padding-top: 3px;
                padding-left: 1px;
            }
        ''')
        self.btn_test.clicked.connect(self.test_ir_signal.emit)
        btn_center_hbox.addWidget(self.btn_test)
        btn_center_hbox.addStretch(1)
        trigger_layout.addLayout(btn_center_hbox)

        self.btn_settings = QPushButton('⚙️ 하드웨어 설정')
        self.btn_settings.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_settings.setStyleSheet('''
            QPushButton {
                background-color: #37474F;
                color: #ECEFF1;
                font-weight: bold;
                padding: 10px;
                font-size: 13px;
                border-radius: 6px;
                border: 1px solid #455A64;
            }
            QPushButton:hover {
                background-color: #455A64;
                color: #ffffff;
                border-color: #546E7A;
            }
            QPushButton:pressed {
                background-color: #263238;
                padding-top: 11px;
                padding-bottom: 9px;
            }
        ''')
        self.btn_settings.clicked.connect(self.open_settings_dialog)

        vbox.addStretch(1) # Push both trigger_panel and settings button to the bottom!
        vbox.addWidget(trigger_panel)
        vbox.addSpacing(8) # Sleek gap between simulation panel and settings button
        vbox.addWidget(self.btn_settings)

        self.layout.addLayout(vbox, 0, 1)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.timer.start(1000)

    def _on_virtual_delay_slider_changed(self, value: int):
        self.virtual_delay_sec = float(value)
        self.lbl_virtual_delay.setText(f'적용 지연: +{value}초')
        self.virtual_delay_changed.emit(self.virtual_delay_sec)

    def _on_reset_delay_clicked(self):
        self.slider_delay.setValue(0)

    def init_column3(self):
        vbox = QVBoxLayout()

        self.table_history = QTableWidget(0, 5)
        self.table_history.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table_history.setHorizontalHeaderLabels(['ID', '발생 시간', '결과', '지연(초)', '상세 보기'])
        self.table_history.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table_history.setStyleSheet('background-color: #2b2b2b; color: white;')

        vbox.addWidget(QLabel('<b>작업 히스토리</b>'))
        vbox.addWidget(self.table_history, stretch=3)

        self.layout.addLayout(vbox, 0, 2)



    def update_time(self):
        self.lbl_sys_time.setText(time.strftime('%Y-%m-%d %H:%M:%S'))

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

    def show_event_detail_dialog(self, event_id):
        if not self.run_folder:
            QMessageBox.warning(self, '오류', '로그 저장 폴더가 설정되지 않았습니다.')
            return

        event_folder = os.path.join(self.run_folder, str(event_id))
        img_path = os.path.join(event_folder, "image.jpg")
        res_path = os.path.join(event_folder, "response.txt")

        if not os.path.exists(res_path):
            QMessageBox.information(self, '정보', f'ID {event_id}번 작업의 분석 결과 파일(response.txt)이 아직 생성되지 않았거나 존재하지 않습니다.')
            return

        # Load response
        try:
            with open(res_path, "r", encoding="utf-8") as f:
                response_text = f.read()
        except Exception as e:
            response_text = f"응답 파일을 읽는 중 오류 발생: {e}"

        # Load image if exists
        frame_img = None
        if os.path.exists(img_path):
            try:
                frame_img = cv2.imread(img_path)
            except Exception as e:
                print(f"이미지 파일 로드 실패: {e}")

        details = {
            "image": frame_img,
            "response": response_text
        }

        dialog = EventDetailDialog(event_id, details, self)
        dialog.exec()

    def handle_settings_apply_result(self, success: bool, message: str, applied_values: dict | None):
        if self._settings_dialog is not None:
            self._settings_dialog.handle_apply_result(success, message, applied_values)

    def set_runtime_settings(self, settings: dict):
        self.runtime_settings = dict(settings)

    @pyqtSlot(object)
    def set_live_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix_h = self.lbl_live_feed.height()
        pix_w = self.lbl_live_feed.width()
        if pix_h > 0 and pix_w > 0:
            pix = QPixmap.fromImage(qimg).scaled(pix_w, pix_h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.lbl_live_feed.setPixmap(pix)

    @pyqtSlot(object)
    def set_ai_image(self, frame):
        if frame is None:
            self.last_ai_frame = None
            return
        self.last_ai_frame = frame.copy()
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qimg = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        pix_h = self.lbl_ai_img.height()
        pix_w = self.lbl_ai_img.width()
        if pix_h > 0 and pix_w > 0:
            pix = QPixmap.fromImage(qimg).scaled(pix_w, pix_h, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
            self.lbl_ai_img.setPixmap(pix)

    @pyqtSlot(int, str)
    def add_classification_started(self, event_id, time_str):
        active_virtual_delay = self.virtual_delay_sec

        # Reset virtual delay slider to 0 when an object enters
        self.slider_delay.blockSignals(True)
        self.slider_delay.setValue(0)
        self.slider_delay.blockSignals(False)
        self.virtual_delay_sec = 0.0
        self.lbl_virtual_delay.setText('적용 지연: +0초')

        # Reset center displays to waiting state with centered emoji
        self.lbl_ai_img.setPixmap(QPixmap())
        self.lbl_ai_img.setStyleSheet('background-color: #222222; border: 1px solid #555; font-size: 14pt;')
        self.lbl_ai_img.setText('⏳')
        self.txt_ai_res.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.txt_ai_res.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 14pt;')
        self.txt_ai_res.setText('⏳')

        row = self.table_history.rowCount()
        self.table_history.insertRow(row)

        item_id = QTableWidgetItem(str(event_id))
        item_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 0, item_id)

        item_time = QTableWidgetItem(time_str)
        item_time.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 1, item_time)

        item_result = QTableWidgetItem('분류 중...')
        item_result.setForeground(QColor('#AAAAAA'))  # Muted grey
        item_result.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 2, item_result)

        if active_virtual_delay > 0.0:
            item_latency = QTableWidgetItem(f'지연 대기 (+{int(active_virtual_delay)}초)...')
            item_latency.setForeground(QColor('#FF9800')) # Warm orange/amber for active delay
        else:
            item_latency = QTableWidgetItem('분류 중...')
            item_latency.setForeground(QColor('#AAAAAA'))
        item_latency.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 3, item_latency)

        item_detail = QTableWidgetItem('대기 중')
        item_detail.setForeground(QColor('#AAAAAA'))
        item_detail.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 4, item_detail)

        self.event_row_map[event_id] = row
        self.table_history.scrollToBottom()

    @pyqtSlot(int, int, float, tuple, str)
    def add_classification_result(self, event_id, result_flag, latency, coeffs, raw_text):
        # Align left/top and add nice padding for actual response display
        self.txt_ai_res.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        self.txt_ai_res.setStyleSheet('background-color: #2b2b2b; color: #FFFFFF; border: 1px solid #555; font-size: 11pt; padding: 8px;')
        self.txt_ai_res.setText(raw_text)

        self.latency_data.append(latency)
        if len(self.latency_data) > 30:
            self.latency_data.pop(0)

        # Clear last_ai_frame
        self.last_ai_frame = None

        row = self.event_row_map.get(event_id)
        if row is None:
            row = self.table_history.rowCount()
            self.table_history.insertRow(row)

            item_id = QTableWidgetItem(str(event_id))
            item_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table_history.setItem(row, 0, item_id)

            item_time = QTableWidgetItem(time.strftime('%H:%M:%S'))
            item_time.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table_history.setItem(row, 1, item_time)

        result_text = '분류' if result_flag == 1 else '미분류'
        item_result = QTableWidgetItem(result_text)
        item_result.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        if result_flag == 1:
            item_result.setForeground(QColor('#FFC107'))  # Gold/Yellow
        else:
            item_result.setForeground(QColor('#03A9F4'))  # Light Blue

        self.table_history.setItem(row, 2, item_result)
        
        item_latency = QTableWidgetItem(f'{latency:.3f}')
        item_latency.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 3, item_latency)

        # Add interactive view button to column 4
        btn_detail = QPushButton('🔍')
        btn_detail.setCursor(Qt.CursorShape.PointingHandCursor)
        
        if result_flag == 1:
            bg_color = '#B8860B'     # Toned-down gold
            hover_color = '#DAA520'  # Slightly brighter gold
            pressed_color = '#996515'# Darker gold
        else:
            bg_color = '#1F4E79'     # Toned-down blue
            hover_color = '#2C6B9E'  # Slightly brighter blue
            pressed_color = '#133553'# Darker blue

        btn_detail.setStyleSheet(
            f'QPushButton {{'
            f'background-color: {bg_color}; '
            f'color: white; '
            f'font-size: 12px; '
            f'border: none; '
            f'border-radius: 4px; '
            f'padding: 3px;'
            f'}}'
            f'QPushButton:hover {{background-color: {hover_color};}}'
            f'QPushButton:pressed {{background-color: {pressed_color};}}'
        )
        btn_detail.clicked.connect(lambda _, eid=event_id: self.show_event_detail_dialog(eid))
        self.table_history.setCellWidget(row, 4, btn_detail)
        
        self.table_history.scrollToBottom()

    @pyqtSlot(float, int)
    def add_missed_result(self, latency, classify_flag):
        self.latency_data.append(latency)
        if len(self.latency_data) > 30:
            self.latency_data.pop(0)

        row = self.table_history.rowCount()
        self.table_history.insertRow(row)

        item_id = QTableWidgetItem('N/A')
        item_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 0, item_id)

        item_time = QTableWidgetItem(time.strftime('%H:%M:%S'))
        item_time.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 1, item_time)

        if classify_flag == 1:
            item_result = QTableWidgetItem('⚠ 미분류 (스텝 도달 초과, 분류 필요했음)')
            item_result.setForeground(QColor('#FF5252'))
        else:
            item_result = QTableWidgetItem('스텝 도달 초과 (패스, 영향 없음)')
            item_result.setForeground(QColor('#888888'))
        item_result.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 2, item_result)

        item_latency = QTableWidgetItem(f'{latency:.3f}')
        item_latency.setForeground(QColor('#FF5252' if classify_flag == 1 else '#888888'))
        item_latency.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 3, item_latency)

        item_detail = QTableWidgetItem('기록 없음')
        item_detail.setForeground(QColor('#888888'))
        item_detail.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_history.setItem(row, 4, item_detail)

        self.table_history.scrollToBottom()

    @pyqtSlot(int)
    def update_queue_count(self, count):
        pass

    @pyqtSlot(bool, bool)
    def update_connection_status(self, cam_ok, ard_ok):
        qss_cam = (
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #80FF80, stop:0.4 #00E676, stop:1 #004D20);'
            if cam_ok else
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
        )
        qss_ard = (
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #80FF80, stop:0.4 #00E676, stop:1 #004D20);'
            if ard_ok else
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
        )
        self.lbl_cam_led.setStyleSheet(f'{qss_cam} border: 2px solid #111111; border-radius: 7px;')
        self.lbl_ard_led.setStyleSheet(f'{qss_ard} border: 2px solid #111111; border-radius: 7px;')

    def update_api_status(self, api_ok):
        qss_api = (
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #80FF80, stop:0.4 #00E676, stop:1 #004D20);'
            if api_ok else
            'background-color: qradialgradient(cx:0.35, cy:0.35, radius:0.8, fx:0.35, fy:0.35, stop:0 #FF8080, stop:0.4 #D50000, stop:1 #4D0000);'
        )
        self.lbl_api_led.setStyleSheet(f'{qss_api} border: 2px solid #111111; border-radius: 7px;')

    def reset_status_labels(self):
        """결과 표시 후 라벨들을 초기 대기 상태로 복구 (미사용)"""
        pass

    def set_hardware_info(self, target_deg, belt_speed, microstep,
                          full_steps_rev=None, ir_distance=None,
                          target_steps=None, serial_port=None,
                          baud_rate=None, camera_index=None, ai_model=None):
        pass
