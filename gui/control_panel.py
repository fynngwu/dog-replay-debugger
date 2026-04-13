from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class ControlPanel(QWidget):
    """Left-side control panel adapted for dog_fifo_backend."""

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        file_box = QGroupBox('Replay assets')
        file_layout = QFormLayout(file_box)
        file_layout.setContentsMargins(12, 12, 12, 12)
        file_layout.setSpacing(8)

        self.csv_path = QLineEdit()
        self.xml_path = QLineEdit()
        self.csv_browse = QPushButton('Open CSV')
        self.xml_browse = QPushButton('Open MuJoCo XML')
        self.csv_load = QPushButton('Load')
        self.xml_load = QPushButton('Load')

        row1 = QHBoxLayout(); row1.setSpacing(8)
        row1.addWidget(self.csv_path)
        row1.addWidget(self.csv_browse)
        row1.addWidget(self.csv_load)
        csv_wrap = QWidget(); csv_wrap.setLayout(row1)

        row2 = QHBoxLayout(); row2.setSpacing(8)
        row2.addWidget(self.xml_path)
        row2.addWidget(self.xml_browse)
        row2.addWidget(self.xml_load)
        xml_wrap = QWidget(); xml_wrap.setLayout(row2)

        file_layout.addRow('CSV', csv_wrap)
        file_layout.addRow('MuJoCo XML', xml_wrap)
        root.addWidget(file_box)

        robot_box = QGroupBox('dog_fifo_backend')
        robot_layout = QGridLayout(robot_box)
        robot_layout.setContentsMargins(12, 12, 12, 12)
        robot_layout.setSpacing(8)

        self.host = QLineEdit('10.20.127.185')
        self.cmd_port = QLineEdit('47001')
        self.state_port = QLineEdit('47002')
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.setObjectName('connect_btn')
        self.init_btn = QPushButton('Init')
        self.disable_btn = QPushButton('Disable')

        robot_layout.addWidget(QLabel('Host'), 0, 0)
        robot_layout.addWidget(self.host, 0, 1, 1, 3)
        robot_layout.addWidget(QLabel('Cmd'), 1, 0)
        robot_layout.addWidget(self.cmd_port, 1, 1)
        robot_layout.addWidget(QLabel('State'), 1, 2)
        robot_layout.addWidget(self.state_port, 1, 3)
        self.set_all_zero_btn = QPushButton('Set All Zero')

        robot_layout.addWidget(self.connect_btn, 2, 0, 1, 2)
        robot_layout.addWidget(self.init_btn, 2, 2)
        robot_layout.addWidget(self.disable_btn, 2, 3)
        robot_layout.addWidget(self.set_all_zero_btn, 3, 0, 1, 4)
        root.addWidget(robot_box)

        remote_box = QGroupBox('Remote backend ops (SSH)')
        remote_layout = QVBoxLayout(remote_box)
        remote_layout.setContentsMargins(12, 12, 12, 12)
        remote_layout.setSpacing(8)

        remote_hint = QLabel('Uses ares@<Host>. Start command: cd ~/backend && nohup ./build/daemon ...')
        remote_hint.setWordWrap(True)
        remote_layout.addWidget(remote_hint)

        remote_btn_row = QHBoxLayout()
        self.kill_backend_btn = QPushButton('SSH Kill dog')
        self.start_backend_btn = QPushButton('SSH Start daemon')
        remote_btn_row.addWidget(self.kill_backend_btn)
        remote_btn_row.addWidget(self.start_backend_btn)
        remote_layout.addLayout(remote_btn_row)
        root.addWidget(remote_box)

        mit_box = QGroupBox('MIT params')
        mit_layout = QGridLayout(mit_box)
        mit_layout.setContentsMargins(12, 12, 12, 12)
        mit_layout.setSpacing(8)

        self.kp_spin = QDoubleSpinBox(); self.kp_spin.setRange(0.0, 500.0); self.kp_spin.setDecimals(3); self.kp_spin.setValue(40.0)
        self.kd_spin = QDoubleSpinBox(); self.kd_spin.setRange(0.0, 20.0); self.kd_spin.setDecimals(3); self.kd_spin.setValue(0.5)
        self.vel_limit_spin = QDoubleSpinBox(); self.vel_limit_spin.setRange(0.0, 200.0); self.vel_limit_spin.setDecimals(3); self.vel_limit_spin.setValue(44.0)
        self.torque_limit_spin = QDoubleSpinBox(); self.torque_limit_spin.setRange(0.0, 100.0); self.torque_limit_spin.setDecimals(3); self.torque_limit_spin.setValue(17.0)
        self.apply_mit_btn = QPushButton('Apply MIT Params')

        mit_layout.addWidget(QLabel('KP'), 0, 0)
        mit_layout.addWidget(self.kp_spin, 0, 1)
        mit_layout.addWidget(QLabel('KD'), 0, 2)
        mit_layout.addWidget(self.kd_spin, 0, 3)
        mit_layout.addWidget(QLabel('Vel limit'), 1, 0)
        mit_layout.addWidget(self.vel_limit_spin, 1, 1)
        mit_layout.addWidget(QLabel('Torque limit'), 1, 2)
        mit_layout.addWidget(self.torque_limit_spin, 1, 3)
        mit_layout.addWidget(self.apply_mit_btn, 2, 0, 1, 4)
        root.addWidget(mit_box)

        replay_box = QGroupBox('Replay control')
        replay_layout = QGridLayout(replay_box)
        replay_layout.setContentsMargins(12, 12, 12, 12)
        replay_layout.setSpacing(8)

        self.start_btn = QPushButton('Start')
        self.start_btn.setObjectName('start_btn')
        self.stop_btn = QPushButton('Stop')
        self.stop_btn.setObjectName('stop_btn')
        self.prev_btn = QPushButton('Prev')
        self.step_btn = QPushButton('Step')

        self.frame_spin = QSpinBox()
        self.frame_spin.setRange(0, 0)
        self.frame_spin.setMinimumWidth(80)

        self.frame_slider = QSlider(Qt.Orientation.Horizontal)
        self.frame_slider.setRange(0, 0)

        self.speed_input = QLineEdit('1.0')

        replay_layout.addWidget(self.start_btn, 0, 0)
        replay_layout.addWidget(self.stop_btn, 0, 1)
        replay_layout.addWidget(self.prev_btn, 0, 2)
        replay_layout.addWidget(self.step_btn, 0, 3)
        replay_layout.addWidget(QLabel('Frame'), 1, 0)
        replay_layout.addWidget(self.frame_spin, 1, 1)
        replay_layout.addWidget(self.frame_slider, 1, 2, 1, 2)
        replay_layout.addWidget(QLabel('Speed'), 2, 0)
        replay_layout.addWidget(self.speed_input, 2, 1)
        replay_layout.addWidget(QLabel('x replay'), 2, 2, 1, 2)
        root.addWidget(replay_box)

        record_box = QGroupBox('Recording')
        record_layout = QHBoxLayout(record_box)
        record_layout.setContentsMargins(12, 12, 12, 12)
        record_layout.setSpacing(8)

        self.record_btn = QPushButton('Record')
        self.record_btn.setObjectName('record_btn')
        self.stop_record_btn = QPushButton('Stop Record')
        self.stop_record_btn.setObjectName('stop_record_btn')
        self.stop_record_btn.setEnabled(False)

        record_layout.addWidget(self.record_btn)
        record_layout.addWidget(self.stop_record_btn)
        root.addWidget(record_box)

        root.addStretch(1)
