"""Status panel component with visual indicators."""

from __future__ import annotations

from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QWidget


class StatusPanel(QFrame):
    """Compact top-level status summary for replay and backend state."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.setObjectName('status_panel')

        self.setStyleSheet(self._get_status_style())
        self._setup_ui()

    def _get_status_style(self) -> str:
        from gui.styles import STATUS_PANEL_STYLE
        return STATUS_PANEL_STYLE

    def _setup_ui(self) -> None:
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(12)

        fields = [
            ('Frame', 'frame_label', '-- / --'),
            ('Replay', 'playing_label', 'Stopped'),
            ('Speed', 'speed_label', '1.00x'),
            ('Robot', 'robot_conn_label', 'Disconnected'),
            ('TX/RX', 'robot_rates_label', '-- / -- Hz'),
            ('State age', 'robot_age_label', '--'),
            ('Enabled', 'enabled_label', '--'),
            ('Busy', 'busy_label', '--'),
            ('Queue', 'queue_label', '--'),
            ('KP/KD', 'mit_label', '-- / --'),
            ('MuJoCo', 'mujoco_status_label', 'Not Loaded'),
            ('Viewer', 'viewer_label', 'Stopped'),
        ]
        for i, (title, attr, default) in enumerate(fields):
            row = i // 6
            col = (i % 6) * 2
            layout.addWidget(QLabel(f'{title}:'), row, col)
            label = QLabel(default)
            label.setObjectName('status_value')
            setattr(self, attr, label)
            layout.addWidget(label, row, col + 1)

    def update_status(self, snapshot) -> None:  # noqa: ANN001
        if snapshot is None:
            return

        max_idx = max(0, snapshot.total_frames - 1)
        self.frame_label.setText(f'{snapshot.cursor} / {max_idx}')
        self.speed_label.setText(f'{snapshot.playback_speed:.2f}x')
        self.playing_label.setText('Playing' if snapshot.playing else 'Stopped')
        self.robot_conn_label.setText('Connected' if snapshot.robot_connected else 'Disconnected')
        self.robot_rates_label.setText(
            f'{snapshot.robot_tx_hz:.1f} / {snapshot.robot_rx_hz:.1f} Hz' if snapshot.robot_connected else '-- / -- Hz'
        )
        self.robot_age_label.setText('--' if snapshot.backend_state_age_s is None else f'{snapshot.backend_state_age_s:.3f}s')

        backend_state = snapshot.backend_state
        if backend_state is None:
            self.enabled_label.setText('--')
            self.busy_label.setText('--')
            self.queue_label.setText('--')
            self.mit_label.setText('-- / --')
        else:
            self.enabled_label.setText('true' if backend_state.enabled else 'false')
            self.busy_label.setText('true' if backend_state.busy else 'false')
            self.queue_label.setText(str(backend_state.queue_size))
            kp = '--' if backend_state.kp is None else f'{backend_state.kp:.3f}'
            kd = '--' if backend_state.kd is None else f'{backend_state.kd:.3f}'
            self.mit_label.setText(f'{kp} / {kd}')

        self.mujoco_status_label.setText('Loaded' if snapshot.mujoco_loaded else 'Not Loaded')
        self.viewer_label.setText('Running' if snapshot.mujoco_viewer_running else 'Stopped')
