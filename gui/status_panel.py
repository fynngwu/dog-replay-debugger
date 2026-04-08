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
        """Get the status panel stylesheet."""
        from gui.styles import STATUS_PANEL_STYLE
        return STATUS_PANEL_STYLE

    def _setup_ui(self) -> None:
        """Setup the status panel UI layout."""
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(12)

        # Frame info
        layout.addWidget(QLabel("Frame:"), 0, 0)
        self.frame_label = QLabel("-- / --")
        self.frame_label.setObjectName("status_value")
        layout.addWidget(self.frame_label, 0, 1)

        # Playing status
        layout.addWidget(QLabel("Status:"), 0, 2)
        self.playing_label = QLabel("Stopped")
        self.playing_label.setObjectName("status_value")
        layout.addWidget(self.playing_label, 0, 3)

        # Robot connection
        layout.addWidget(QLabel("Robot:"), 0, 4)
        self.robot_conn_label = QLabel("Disconnected")
        self.robot_conn_label.setObjectName("status_value_error")
        layout.addWidget(self.robot_conn_label, 0, 5)

        # Robot rates
        layout.addWidget(QLabel("TX/RX:"), 1, 0)
        self.robot_rates_label = QLabel("-- / -- Hz")
        self.robot_rates_label.setObjectName("status_value")
        layout.addWidget(self.robot_rates_label, 1, 1)

        # Robot age
        layout.addWidget(QLabel("Age:"), 1, 2)
        self.robot_age_label = QLabel("--")
        self.robot_age_label.setObjectName("status_value")
        layout.addWidget(self.robot_age_label, 1, 3)

        # MuJoCo status
        layout.addWidget(QLabel("MuJoCo:"), 1, 4)
        self.mujoco_status_label = QLabel("Not Loaded")
        self.mujoco_status_label.setObjectName("status_value_error")
        layout.addWidget(self.mujoco_status_label, 1, 5)

        # MuJoCo apply rate
        layout.addWidget(QLabel("Apply:"), 2, 0)
        self.mujoco_apply_label = QLabel("-- Hz")
        self.mujoco_apply_label.setObjectName("status_value")
        layout.addWidget(self.mujoco_apply_label, 2, 1)

        # MuJoCo age
        layout.addWidget(QLabel("Age:"), 2, 2)
        self.mujoco_age_label = QLabel("--")
        self.mujoco_age_label.setObjectName("status_value")
        layout.addWidget(self.mujoco_age_label, 2, 3)

        # Viewer status
        layout.addWidget(QLabel("Viewer:"), 2, 4)
        self.viewer_label = QLabel("Stopped")
        self.viewer_label.setObjectName("status_value_error")
        layout.addWidget(self.viewer_label, 2, 5)

        # Add column stretch
        layout.setColumnStretch(6, 1)

    def update_status(self, snapshot) -> None:  # noqa: ANN001
        """Update status panel with snapshot data."""
        if snapshot is None:
            return

        # Frame info
        max_idx = max(0, snapshot.total_frames - 1)
        self.frame_label.setText(f"{snapshot.cursor} / {max_idx}")

        # Playing status
        if snapshot.playing:
            self.playing_label.setText("Playing")
            self.playing_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.playing_label.setText("Stopped")
            self.playing_label.setStyleSheet("color: #b0b0c0; font-family: 'Consolas'; font-size: 11px;")

        # Robot connection
        if snapshot.robot_connected:
            self.robot_conn_label.setText("Connected")
            self.robot_conn_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.robot_conn_label.setText("Disconnected")
            self.robot_conn_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")

        # Robot rates
        if snapshot.robot_connected:
            self.robot_rates_label.setText(f"{snapshot.robot_tx_hz:.1f} / {snapshot.robot_rx_hz:.1f} Hz")
        else:
            self.robot_rates_label.setText("-- / -- Hz")

        # Robot age
        if snapshot.robot_state_age_s is not None:
            age = snapshot.robot_state_age_s
            self.robot_age_label.setText(f"{age:.3f}s")
            # Color code based on age
            if age > 0.5:
                self.robot_age_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")
            elif age > 0.2:
                self.robot_age_label.setStyleSheet("color: #ffd740; font-family: 'Consolas'; font-size: 11px;")
            else:
                self.robot_age_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.robot_age_label.setText("--")
            self.robot_age_label.setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 11px;")

        # MuJoCo status
        if snapshot.mujoco_loaded:
            self.mujoco_status_label.setText("Loaded")
            self.mujoco_status_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.mujoco_status_label.setText("Not Loaded")
            self.mujoco_status_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")

        # MuJoCo apply rate
        if snapshot.mujoco_loaded:
            self.mujoco_apply_label.setText(f"{snapshot.mujoco_apply_hz:.1f} Hz")
        else:
            self.mujoco_apply_label.setText("-- Hz")

        # MuJoCo age
        if snapshot.mujoco_state_age_s is not None:
            age = snapshot.mujoco_state_age_s
            self.mujoco_age_label.setText(f"{age:.3f}s")
            # Color code based on age
            if age > 0.1:
                self.mujoco_age_label.setStyleSheet("color: #ff5252; font-family: 'Consolas'; font-size: 11px;")
            elif age > 0.05:
                self.mujoco_age_label.setStyleSheet("color: #ffd740; font-family: 'Consolas'; font-size: 11px;")
            else:
                self.mujoco_age_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.mujoco_age_label.setText("--")
            self.mujoco_age_label.setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 11px;")

        # Viewer status
        if snapshot.mujoco_viewer_running:
            self.viewer_label.setText("Running")
            self.viewer_label.setStyleSheet("color: #69f0ae; font-family: 'Consolas'; font-size: 11px;")
        else:
            self.viewer_label.setText("Stopped")
            self.viewer_label.setStyleSheet("color: #606070; font-family: 'Consolas'; font-size: 11px;")
