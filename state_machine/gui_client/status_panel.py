from __future__ import annotations

from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtWidgets import (
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
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


class StatusPanel(QWidget):
    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        self.mode_label = QLabel("Mode: ---")
        self.mode_label.setObjectName("mode_label")
        root.addWidget(self.mode_label)

        splitter = QSplitter(Qt.Orientation.Vertical)

        joint_box = QGroupBox("Joint States")
        joint_layout = QVBoxLayout(joint_box)
        joint_layout.setContentsMargins(12, 12, 12, 12)
        joint_layout.setSpacing(4)

        self.joint_table = QTableWidget(12, 7)
        self.joint_table.setHorizontalHeaderLabels(
            ["Joint", "Target (rad)", "Robot Pos (rad)", "Mujoco Pos (rad)",
             "Robot Vel (rad/s)", "Mujoco Vel (rad/s)", ""]
        )
        self.joint_table.horizontalHeader().setStretchLastSection(True)
        self.joint_table.setColumnWidth(0, 72)
        for col in range(1, 6):
            self.joint_table.setColumnWidth(col, 100)
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.joint_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)

        for i, name in enumerate(JOINT_NAMES):
            self.joint_table.setItem(i, 0, QTableWidgetItem(name))
            self.joint_table.setItem(i, 1, QTableWidgetItem("0.0000"))
            self.joint_table.setItem(i, 2, QTableWidgetItem("0.0000"))
            self.joint_table.setItem(i, 3, QTableWidgetItem("0.0000"))
            self.joint_table.setItem(i, 4, QTableWidgetItem("0.0000"))
            self.joint_table.setItem(i, 5, QTableWidgetItem("0.0000"))

        joint_layout.addWidget(self.joint_table)
        splitter.addWidget(joint_box)

        imu_container = QWidget()
        imu_h_layout = QHBoxLayout(imu_container)
        imu_h_layout.setContentsMargins(0, 0, 0, 0)
        imu_h_layout.setSpacing(8)

        self.imu_labels: dict[str, QLabel] = {}
        imu_fields = [
            ("Gyro X", "gyro_x"),
            ("Gyro Y", "gyro_y"),
            ("Gyro Z", "gyro_z"),
            ("Gravity X", "grav_x"),
            ("Gravity Y", "grav_y"),
            ("Gravity Z", "grav_z"),
        ]
        for prefix, title in [("robot", "Robot IMU"), ("mujoco", "MuJoCo IMU")]:
            imu_box = QGroupBox(title)
            imu_layout = QGridLayout(imu_box)
            imu_layout.setContentsMargins(12, 12, 12, 12)
            imu_layout.setSpacing(6)
            for row, (label, key) in enumerate(imu_fields):
                name_lbl = QLabel(label)
                name_lbl.setObjectName("info_label")
                val_lbl = QLabel("0.0000")
                val_lbl.setObjectName("val")
                self.imu_labels[f"{prefix}_{key}"] = val_lbl
                imu_layout.addWidget(name_lbl, row, 0)
                imu_layout.addWidget(val_lbl, row, 1)
            imu_h_layout.addWidget(imu_box)

        splitter.addWidget(imu_container)

        log_box = QGroupBox("Log")
        log_layout = QVBoxLayout(log_box)
        log_layout.setContentsMargins(12, 12, 12, 12)
        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        log_layout.addWidget(self.log_edit)
        splitter.addWidget(log_box)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setStretchFactor(2, 2)
        root.addWidget(splitter)

    def update_mode(self, mode_str: str) -> None:
        self.mode_label.setText(f"Mode: {mode_str}")

    def update_joints(self, target: list[float], robot_pos: list[float], robot_vel: list[float], mujoco_pos: list[float], mujoco_vel: list[float]) -> None:
        with QSignalBlocker(self.joint_table):
            for i in range(12):
                self.joint_table.item(i, 1).setText(f"{target[i]:.4f}")
                self.joint_table.item(i, 2).setText(f"{robot_pos[i]:.4f}")
                self.joint_table.item(i, 3).setText(f"{mujoco_pos[i]:.4f}")
                self.joint_table.item(i, 4).setText(f"{robot_vel[i]:.4f}")
                self.joint_table.item(i, 5).setText(f"{mujoco_vel[i]:.4f}")

    def update_imu(self, robot_gyro: list[float], robot_gravity: list[float], mujoco_gyro: list[float], mujoco_gravity: list[float]) -> None:
        self.imu_labels["robot_gyro_x"].setText(f"{robot_gyro[0]:.4f}")
        self.imu_labels["robot_gyro_y"].setText(f"{robot_gyro[1]:.4f}")
        self.imu_labels["robot_gyro_z"].setText(f"{robot_gyro[2]:.4f}")
        self.imu_labels["robot_grav_x"].setText(f"{robot_gravity[0]:.4f}")
        self.imu_labels["robot_grav_y"].setText(f"{robot_gravity[1]:.4f}")
        self.imu_labels["robot_grav_z"].setText(f"{robot_gravity[2]:.4f}")
        self.imu_labels["mujoco_gyro_x"].setText(f"{mujoco_gyro[0]:.4f}")
        self.imu_labels["mujoco_gyro_y"].setText(f"{mujoco_gyro[1]:.4f}")
        self.imu_labels["mujoco_gyro_z"].setText(f"{mujoco_gyro[2]:.4f}")
        self.imu_labels["mujoco_grav_x"].setText(f"{mujoco_gravity[0]:.4f}")
        self.imu_labels["mujoco_grav_y"].setText(f"{mujoco_gravity[1]:.4f}")
        self.imu_labels["mujoco_grav_z"].setText(f"{mujoco_gravity[2]:.4f}")

    def append_log(self, msg: str) -> None:
        self.log_edit.appendPlainText(msg)
        self.log_edit.verticalScrollBar().setValue(
            self.log_edit.verticalScrollBar().maximum()
        )
