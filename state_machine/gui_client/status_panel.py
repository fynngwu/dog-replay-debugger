from __future__ import annotations

import csv

from PySide6.QtCore import QSignalBlocker, Qt
from PySide6.QtWidgets import (
    QGroupBox,
    QGridLayout,
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

        self.joint_table = QTableWidget(12, 4)
        self.joint_table.setHorizontalHeaderLabels(
            ["Joint", "Position (rad)", "Velocity (rad/s)", ""]
        )
        self.joint_table.horizontalHeader().setStretchLastSection(True)
        self.joint_table.setColumnWidth(0, 80)
        self.joint_table.setColumnWidth(1, 120)
        self.joint_table.setColumnWidth(2, 120)
        self.joint_table.verticalHeader().setVisible(False)
        self.joint_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.joint_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)

        for i, name in enumerate(JOINT_NAMES):
            self.joint_table.setItem(i, 0, QTableWidgetItem(name))
            self.joint_table.setItem(i, 1, QTableWidgetItem("0.0000"))
            self.joint_table.setItem(i, 2, QTableWidgetItem("0.0000"))

        joint_layout.addWidget(self.joint_table)
        splitter.addWidget(joint_box)

        imu_box = QGroupBox("IMU")
        imu_layout = QGridLayout(imu_box)
        imu_layout.setContentsMargins(12, 12, 12, 12)
        imu_layout.setSpacing(6)

        self.imu_labels: dict[str, QLabel] = {}
        imu_fields = [
            ("Gyro X", "gyro_x"),
            ("Gyro Y", "gyro_y"),
            ("Gyro Z", "gyro_z"),
            ("Gravity X", "grav_x"),
            ("Gravity Y", "grav_y"),
            ("Gravity Z", "grav_z"),
        ]
        for row, (label, key) in enumerate(imu_fields):
            name_lbl = QLabel(label)
            name_lbl.setObjectName("info_label")
            val_lbl = QLabel("0.0000")
            val_lbl.setObjectName("val")
            self.imu_labels[key] = val_lbl
            imu_layout.addWidget(name_lbl, row, 0)
            imu_layout.addWidget(val_lbl, row, 1)

        splitter.addWidget(imu_box)

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

    def update_joints(self, position: list[float], velocity: list[float]) -> None:
        for i in range(12):
            with QSignalBlocker(self.joint_table.item(i, 1)):
                self.joint_table.item(i, 1).setText(f"{position[i]:.4f}")
            with QSignalBlocker(self.joint_table.item(i, 2)):
                self.joint_table.item(i, 2).setText(f"{velocity[i]:.4f}")

    def update_imu(self, gyro: list[float], gravity: list[float]) -> None:
        self.imu_labels["gyro_x"].setText(f"{gyro[0]:.4f}")
        self.imu_labels["gyro_y"].setText(f"{gyro[1]:.4f}")
        self.imu_labels["gyro_z"].setText(f"{gyro[2]:.4f}")
        self.imu_labels["grav_x"].setText(f"{gravity[0]:.4f}")
        self.imu_labels["grav_y"].setText(f"{gravity[1]:.4f}")
        self.imu_labels["grav_z"].setText(f"{gravity[2]:.4f}")

    def append_log(self, msg: str) -> None:
        self.log_edit.appendPlainText(msg)
        self.log_edit.verticalScrollBar().setValue(
            self.log_edit.verticalScrollBar().maximum()
        )


def load_csv_frames(path: str) -> tuple[bool, str, list[list[float]]]:
    try:
        with open(path, newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            frames = []
            for row in reader:
                if len(row) < 13:
                    continue
                try:
                    vals = [float(x) for x in row[1:13]]
                    if len(vals) == 12:
                        frames.append(vals)
                except ValueError:
                    continue
        return True, f"loaded {len(frames)} frames", frames
    except Exception as e:
        return False, str(e), []
