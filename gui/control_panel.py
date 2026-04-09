from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QComboBox,
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
    """Control panel for robot and replay controls."""

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        # File section
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

        row1 = QHBoxLayout()
        row1.setSpacing(8)
        row1.addWidget(self.csv_path)
        row1.addWidget(self.csv_browse)
        row1.addWidget(self.csv_load)

        row2 = QHBoxLayout()
        row2.setSpacing(8)
        row2.addWidget(self.xml_path)
        row2.addWidget(self.xml_browse)
        row2.addWidget(self.xml_load)

        csv_wrap = QWidget(); csv_wrap.setLayout(row1)
        xml_wrap = QWidget(); xml_wrap.setLayout(row2)

        file_layout.addRow('CSV', csv_wrap)
        file_layout.addRow('MuJoCo XML', xml_wrap)
        root.addWidget(file_box)

        # Robot section
        robot_box = QGroupBox('Robot')
        robot_layout = QGridLayout(robot_box)
        robot_layout.setContentsMargins(12, 12, 12, 12)
        robot_layout.setSpacing(8)

        self.host = QLineEdit('10.20.127.185')
        self.cmd_port = QLineEdit('47001')
        self.state_port = QLineEdit('47002')
        self.connect_btn = QPushButton('Connect')
        self.connect_btn.setObjectName('connect_btn')
        self.ping_btn = QPushButton('Ping')
        self.init_btn = QPushButton('Init')
        self.enable_btn = QPushButton('Enable')
        self.disable_btn = QPushButton('Disable')

        robot_layout.addWidget(QLabel('Host'), 0, 0)
        robot_layout.addWidget(self.host, 0, 1)
        robot_layout.addWidget(QLabel('Cmd'), 0, 2)
        robot_layout.addWidget(self.cmd_port, 0, 3)
        robot_layout.addWidget(QLabel('State'), 0, 4)
        robot_layout.addWidget(self.state_port, 0, 5)

        robot_layout.addWidget(self.connect_btn, 1, 0, 1, 2)
        robot_layout.addWidget(self.ping_btn, 1, 2)
        robot_layout.addWidget(self.init_btn, 1, 3)
        robot_layout.addWidget(self.enable_btn, 1, 4)
        robot_layout.addWidget(self.disable_btn, 1, 5)

        root.addWidget(robot_box)

        # Replay section
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

        self.speed_combo = QComboBox()
        self.speed_combo.setEditable(True)
        self.speed_combo.addItems(['0.1', '0.25', '0.5', '1.0', '2.0', '4.0'])
        self.speed_combo.setCurrentText('1.0')

        replay_layout.addWidget(self.start_btn, 0, 0)
        replay_layout.addWidget(self.stop_btn, 0, 1)
        replay_layout.addWidget(self.prev_btn, 0, 2)
        replay_layout.addWidget(self.step_btn, 0, 3)

        replay_layout.addWidget(QLabel('Frame'), 1, 0)
        replay_layout.addWidget(self.frame_spin, 1, 1)
        replay_layout.addWidget(self.frame_slider, 1, 2, 1, 2)
        replay_layout.addWidget(QLabel('Speed'), 2, 0)
        replay_layout.addWidget(self.speed_combo, 2, 1)
        replay_layout.addWidget(QLabel('x at start'), 2, 2, 1, 2)
        root.addWidget(replay_box)

        # Recording section
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
