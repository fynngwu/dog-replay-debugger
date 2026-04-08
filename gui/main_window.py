from __future__ import annotations

import sys

from PySide6.QtCore import QSignalBlocker, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from gui.control_panel import ControlPanel
from gui.curves_panel import CurvesPanel
from gui.joint_dashboard import JointDashboard
from gui.status_panel import StatusPanel
from gui.styles import MODERN_STYLE
from replay_core.replay_engine import ReplayEngine


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Dog Replay Debugger')
        self.resize(1600, 1000)
        self.setStyleSheet(MODERN_STYLE)

        self.engine = ReplayEngine()
        self._last_log_count = 0

        # Create UI components
        self.controls = ControlPanel()
        self.dashboard = JointDashboard()
        self.status_panel = StatusPanel()
        self.curves = CurvesPanel()
        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)

        # Setup main layout
        self._setup_ui()
        self._wire_actions()
        self._load_default_files()

        # Setup refresh timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(20)

        # Connect dashboard selection to curves
        self.dashboard.joint_selected.connect(self._on_joint_selected)

    def _setup_ui(self) -> None:
        """Setup the main window UI layout."""
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(12)

        # Create splitter for resizable panels
        splitter = QSplitter()
        splitter.setHandleWidth(2)

        # Left panel (controls + logs)
        left_panel = self._create_left_panel()
        splitter.addWidget(left_panel)

        # Right panel (dashboard + status + curves)
        right_panel = self._create_right_panel()
        splitter.addWidget(right_panel)

        # Set splitter sizes (30% left, 70% right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([400, 1200])

        main_layout.addWidget(splitter)

    def _create_left_panel(self) -> QWidget:
        """Create the left control panel."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # Control panel
        layout.addWidget(self.controls)

        # Logs section
        logs_label = QWidget()
        logs_layout = QVBoxLayout(logs_label)
        logs_layout.setContentsMargins(0, 0, 0, 0)
        logs_label.setLayout(logs_layout)

        logs_title = QLabel("System Logs")
        logs_title.setStyleSheet("color: #b0b0c0; font-weight: 600; font-size: 13px; padding: 6px;")
        logs_layout.addWidget(logs_title)
        logs_layout.addWidget(self.logs)

        layout.addWidget(logs_label)

        return panel

    def _create_right_panel(self) -> QWidget:
        """Create the right data panel."""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        # Status panel at the top
        layout.addWidget(self.status_panel)

        # Joint dashboard (main content)
        layout.addWidget(self.dashboard)

        # Curves panel at the bottom (smaller)
        self.curves.setMaximumHeight(250)
        layout.addWidget(self.curves)

        return panel

    def closeEvent(self, event) -> None:  # noqa: N802
        self.engine.close()
        super().closeEvent(event)

    def _wire_actions(self) -> None:
        """Wire up control panel actions."""
        c = self.controls
        c.connect_btn.clicked.connect(self._connect_robot)
        c.ping_btn.clicked.connect(lambda: self.engine.robot_ping())
        c.init_btn.clicked.connect(lambda: self.engine.robot_init())
        c.enable_btn.clicked.connect(lambda: self.engine.robot_enable())
        c.disable_btn.clicked.connect(lambda: self.engine.robot_disable())
        c.start_btn.clicked.connect(self._start)
        c.stop_btn.clicked.connect(self.engine.stop)
        c.prev_btn.clicked.connect(self.engine.prev)
        c.step_btn.clicked.connect(self.engine.step)
        c.frame_spin.valueChanged.connect(self._seek_from_spin)
        c.frame_slider.valueChanged.connect(self._seek_from_slider)

    def _load_default_files(self) -> None:
        """Load default XML and CSV files and display paths."""
        xml_path = '/home/wufy/github_respository/dog_project/dog_replay_debugger/sim_record/leggedrobot_flat_fixed.xml'
        csv_path = '/home/wufy/github_respository/dog_project/dog_replay_debugger/sim_record/output/recording.csv'

        self.controls.xml_path.setText(xml_path)
        self.controls.csv_path.setText(csv_path)

        self.engine.load_mujoco(xml_path, start_viewer=True)
        self.engine.load_csv(csv_path)

    def _connect_robot(self) -> None:
        """Connect to robot with error handling."""
        ok = self.engine.connect_robot(
            self.controls.host.text().strip(),
            int(self.controls.cmd_port.text().strip()),
            int(self.controls.state_port.text().strip()),
        )
        if not ok:
            QMessageBox.warning(
                self,
                'Robot connect failed',
                'Could not connect to robot. See log panel for details.'
            )

    def _start(self) -> None:
        """Start replay."""
        self.engine.start(self.controls.frame_spin.value())

    def _seek_from_spin(self, value: int) -> None:
        """Seek to frame from spinbox."""
        snapshot = self.engine.get_snapshot()
        if snapshot.total_frames == 0 or snapshot.cursor == value:
            return
        self.engine.seek(value)

    def _seek_from_slider(self, value: int) -> None:
        """Seek to frame from slider."""
        snapshot = self.engine.get_snapshot()
        if snapshot.total_frames == 0 or snapshot.cursor == value:
            return
        self.engine.seek(value)

    def _on_joint_selected(self, joint_name: str) -> None:
        """Handle joint card selection to update curves."""
        from replay_core.constants import JOINT_NAMES
        if joint_name in JOINT_NAMES:
            idx = JOINT_NAMES.index(joint_name)
            if hasattr(self.curves, 'joint_combo') and self.curves.joint_combo:
                self.curves.joint_combo.setCurrentIndex(idx)

    def refresh(self) -> None:
        """Refresh UI with current snapshot data."""
        snapshot = self.engine.get_snapshot()
        max_idx = max(0, snapshot.total_frames - 1)

        # Update controls
        self.controls.frame_spin.setMaximum(max_idx)
        self.controls.frame_slider.setMaximum(max_idx)
        with QSignalBlocker(self.controls.frame_spin):
            self.controls.frame_spin.setValue(snapshot.cursor)
        with QSignalBlocker(self.controls.frame_slider):
            self.controls.frame_slider.setValue(snapshot.cursor)

        # Update dashboard
        self.dashboard.update_all(snapshot)

        # Update status panel
        self.status_panel.update_status(snapshot)

        # Update curves
        self.curves.push(snapshot)

        # Update logs
        if len(snapshot.log_lines) != self._last_log_count:
            self.logs.setPlainText('\n'.join(snapshot.log_lines))
            self.logs.verticalScrollBar().setValue(self.logs.verticalScrollBar().maximum())
            self._last_log_count = len(snapshot.log_lines)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()
