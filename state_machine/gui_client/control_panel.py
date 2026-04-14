from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

JOINT_NAMES = [
    "LF_HipA",
    "LR_HipA",
    "RF_HipA",
    "RR_HipA",
    "LF_HipF",
    "LR_HipF",
    "RF_HipF",
    "RR_HipF",
    "LF_Knee",
    "LR_Knee",
    "RF_Knee",
    "RR_Knee",
]


class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        conn_box = QGroupBox("Connection")
        conn_layout = QGridLayout(conn_box)
        conn_layout.setContentsMargins(12, 12, 12, 12)
        conn_layout.setSpacing(8)

        self.host_input = QLineEdit("10.20.127.185")
        self.port_input = QLineEdit("48001")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setObjectName("connect_btn")
        self.disconnect_btn = QPushButton("Disconnect")

        conn_layout.addWidget(QLabel("Host"), 0, 0)
        conn_layout.addWidget(self.host_input, 0, 1)
        conn_layout.addWidget(QLabel("Port"), 0, 2)
        conn_layout.addWidget(self.port_input, 0, 3)
        conn_layout.addWidget(self.connect_btn, 1, 0, 1, 2)
        conn_layout.addWidget(self.disconnect_btn, 1, 2, 1, 2)
        root.addWidget(conn_box)

        mode_box = QGroupBox("Mode")
        mode_layout = QHBoxLayout(mode_box)
        mode_layout.setContentsMargins(12, 12, 12, 12)
        mode_layout.setSpacing(8)

        self.init_btn = QPushButton("Init")
        self.execute_btn = QPushButton("Execute")
        self.execute_btn.setObjectName("start_btn")
        self.policy_btn = QPushButton("Policy")
        self.policy_btn.setObjectName("policy_btn")
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setObjectName("stop_btn")

        mode_layout.addWidget(self.init_btn)
        mode_layout.addWidget(self.execute_btn)
        mode_layout.addWidget(self.policy_btn)
        mode_layout.addWidget(self.stop_btn)
        root.addWidget(mode_box)

        target_box = QGroupBox("Target (12 joints)")
        target_layout = QVBoxLayout(target_box)
        target_layout.setContentsMargins(12, 12, 12, 12)
        target_layout.setSpacing(6)

        self.target_spins: list[QDoubleSpinBox] = []
        for i in range(12):
            row = QHBoxLayout()
            row.setSpacing(6)
            name_label = QLabel(JOINT_NAMES[i])
            name_label.setFixedWidth(72)
            spin = QDoubleSpinBox()
            spin.setRange(-3.14159, 3.14159)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setValue(0.0)
            self.target_spins.append(spin)
            row.addWidget(name_label)
            row.addWidget(spin)
            target_layout.addLayout(row)

        self.send_target_btn = QPushButton("Send Target")
        self.send_target_btn.setObjectName("start_btn")
        target_layout.addWidget(self.send_target_btn)
        root.addWidget(target_box)

        replay_box = QGroupBox("CSV Replay")
        replay_layout = QGridLayout(replay_box)
        replay_layout.setContentsMargins(12, 12, 12, 12)
        replay_layout.setSpacing(8)

        self.csv_path = QLineEdit()
        self.csv_browse = QPushButton("Browse")
        self.replay_start_btn = QPushButton("Start Replay")
        self.replay_start_btn.setObjectName("start_btn")
        self.replay_stop_btn = QPushButton("Stop Replay")
        self.replay_stop_btn.setObjectName("stop_btn")
        self.replay_stop_btn.setEnabled(False)
        self.frame_spin = QSpinBox()
        self.frame_spin.setRange(0, 0)
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(0.1, 5.0)
        self.speed_spin.setValue(1.0)
        self.speed_spin.setDecimals(1)
        self.speed_spin.setSingleStep(0.1)
        self.speed_spin.setSuffix("x")

        csv_row = QHBoxLayout()
        csv_row.setSpacing(8)
        csv_row.addWidget(self.csv_path)
        csv_row.addWidget(self.csv_browse)
        replay_layout.addLayout(csv_row, 0, 0, 1, 3)
        replay_layout.addWidget(QLabel("Frame"), 1, 0)
        replay_layout.addWidget(self.frame_spin, 1, 1)
        replay_layout.addWidget(QLabel("Speed"), 1, 2)
        replay_layout.addWidget(self.speed_spin, 1, 3)
        replay_layout.addWidget(self.replay_start_btn, 2, 0, 1, 2)
        replay_layout.addWidget(self.replay_stop_btn, 2, 2, 1, 2)
        root.addWidget(replay_box)

        mujoco_box = QGroupBox("MuJoCo")
        mujoco_layout = QHBoxLayout(mujoco_box)
        mujoco_layout.setContentsMargins(12, 12, 12, 12)
        mujoco_layout.setSpacing(8)

        self.mujoco_load_btn = QPushButton("Load Model")
        self.mujoco_load_btn.setObjectName("start_btn")
        self.mujoco_close_btn = QPushButton("Close Model")
        self.mujoco_close_btn.setObjectName("stop_btn")
        self.mujoco_close_btn.setEnabled(False)

        mujoco_layout.addWidget(self.mujoco_load_btn)
        mujoco_layout.addWidget(self.mujoco_close_btn)
        root.addWidget(mujoco_box)

        root.addStretch(1)
