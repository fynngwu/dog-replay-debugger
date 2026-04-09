"""Status panel component with visual indicators."""

from __future__ import annotations

from PySide6.QtWidgets import QGridLayout, QLabel, QFrame, QWidget


class StatusPanel(QFrame):
    """A status panel with visual indicators for system status."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName("status_panel")

        self.setStyleSheet(self._get_status_style())
        self._setup_ui()

    def _get_status_style(self) -> str:
        from gui.styles import STATUS_PANEL_STYLE
        return STATUS_PANEL_STYLE

    def _setup_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(12)

        layout.addWidget(QLabel("Frame:"), 0, 0)
        self.frame_label = QLabel("-- / --")
        self.frame_label.setObjectName("status_value")
        layout.addWidget(self.frame_label, 0, 1)

        layout.addWidget(QLabel("Status:"), 0, 2)
        self.playing_label = QLabel("Stopped")
        self.playing_label.setObjectName("status_value")
        layout.addWidget(self.playing_label, 0, 3)

        layout.addWidget(QLabel("Speed:"), 0, 4)
        self.speed_label = QLabel("1.00x")
        self.speed_label.setObjectName("status_value")
        layout.addWidget(self.speed_label, 0, 5)

        layout.addWidget(QLabel("Robot:"), 1, 0)
        self.robot_conn_label = QLabel("Disconnected")
        self.robot_conn_label.setObjectName("status_value_error")
        layout.addWidget(self.robot_conn_label, 1, 1)

        layout.addWidget(QLabel("TX/RX:"), 1, 2)
        self.robot_rates_label = QLabel("-- / -- Hz")
        self.robot_rates_label.setObjectName("status_value")
        layout.addWidget(self.robot_rates_label, 1, 3)

        layout.addWidget(QLabel("Robot age:"), 1, 4)
        self.robot_age_label = QLabel("--")
        self.robot_age_label.setObjectName("status_value")
        layout.addWidget(self.robot_age_label, 1, 5)

        layout.addWidget(QLabel("MuJoCo:"), 2, 0)
        self.mujoco_status_label = QLabel("Not Loaded")
        self.mujoco_status_label.setObjectName("status_value_error")
        layout.addWidget(self.mujoco_status_label, 2, 1)

        layout.addWidget(QLabel("Apply:"), 2, 2)
        self.mujoco_apply_label = QLabel("-- Hz")
        self.mujoco_apply_label.setObjectName("status_value")
        layout.addWidget(self.mujoco_apply_label, 2, 3)

        layout.addWidget(QLabel("Viewer:"), 2, 4)
        self.viewer_label = QLabel("Stopped")
        self.viewer_label.setObjectName("status_value_error")
        layout.addWidget(self.viewer_label, 2, 5)

        layout.setColumnStretch(6, 1)

    def update_status(self, snapshot) -> None:  # noqa: ANN001
        if snapshot is None:
            return

        max_idx = max(0, snapshot.total_frames - 1)
        self.frame_label.setText(f"{snapshot.cursor} / {max_idx}")
        self.speed_label.setText(f"{snapshot.playback_speed:.2f}x")

        if snapshot.playing:
            self.playing_label.setText("Playing")
            self.playing_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.playing_label.setText("Stopped")
            self.playing_label.setStyleSheet("color: #b0b0c0; font-family: 'Consolas'; font-size: 11px;")

        if snapshot.robot_connected:
            self.robot_conn_label.setText("Connected")
            self.robot_conn_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
            self.robot_rates_label.setText(f"{snapshot.robot_tx_hz:.1f} / {snapshot.robot_rx_hz:.1f} Hz")
        else:
            self.robot_conn_label.setText("Disconnected")
            self.robot_conn_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")
            self.robot_rates_label.setText("-- / -- Hz")

        if snapshot.robot_state_age_s is not None:
            age = snapshot.robot_state_age_s
            self.robot_age_label.setText(f"{age:.3f}s")
            color = '#69f0ae' if age <= 0.2 else ('#ffd740' if age <= 0.5 else '#ff5252')
            self.robot_age_label.setStyleSheet(f"color: {color}; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.robot_age_label.setText("--")
            self.robot_age_label.setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 11px;")

        if snapshot.mujoco_loaded:
            self.mujoco_status_label.setText("Loaded")
            self.mujoco_status_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
            self.mujoco_apply_label.setText(f"{snapshot.mujoco_apply_hz:.1f} Hz")
        else:
            self.mujoco_status_label.setText("Not Loaded")
            self.mujoco_status_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")
            self.mujoco_apply_label.setText("-- Hz")

        if snapshot.mujoco_viewer_running:
            self.viewer_label.setText("Running")
            self.viewer_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.viewer_label.setText("Stopped")
            self.viewer_label.setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 11px;")
