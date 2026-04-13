from __future__ import annotations

import csv
import sys
import threading
import time
from pathlib import Path
from typing import List

import numpy as np
from PySide6.QtCore import QSignalBlocker, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from gui.backend_state_panel import BackendStatePanel
from gui.control_panel import ControlPanel
from gui.joint_dashboard import JointDashboard
from gui.knee_debug_panel import KneeDebugPanel
from gui.status_panel import StatusPanel
from gui.styles import MODERN_STYLE
from replay_core.constants import JOINT_NAMES
from replay_core.replay_engine import ReplayEngine


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Dog Replay Debugger')
        self.resize(1680, 1020)
        self.setStyleSheet(MODERN_STYLE)

        self.engine = ReplayEngine()
        self._last_log_count = 0
        self._recording = False
        self._slider_pressed = False
        self._record_start_time: float = 0.0
        self._record_data: List[list[float]] = []

        self.controls = ControlPanel()
        self.dashboard = JointDashboard()
        self.status_panel = StatusPanel()
        self.overview_state = BackendStatePanel('Backend Stream State (Overview tab)')
        self.knee_debug = KneeDebugPanel()
        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)

        self._setup_ui()
        self._wire_actions()
        self._load_default_files()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(20)

    def _setup_ui(self) -> None:
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(12)

        splitter = QSplitter()
        splitter.setHandleWidth(2)
        splitter.addWidget(self._create_left_panel())
        splitter.addWidget(self._create_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 1250])
        main_layout.addWidget(splitter)

    def _create_left_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self.controls)

        logs_label = QWidget()
        logs_layout = QVBoxLayout(logs_label)
        logs_layout.setContentsMargins(0, 0, 0, 0)
        logs_layout.setSpacing(6)
        logs_title = QLabel('System Logs')
        logs_title.setStyleSheet("color: #b0b0c0; font-weight: 600; font-size: 13px; padding: 6px;")
        logs_layout.addWidget(logs_title)
        logs_layout.addWidget(self.logs)
        layout.addWidget(logs_label)
        return panel

    def _create_right_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self.status_panel)

        tabs = QTabWidget()
        tabs.addTab(self._create_overview_tab(), 'Overview')
        tabs.addTab(self.knee_debug, 'Joint Debug')
        layout.addWidget(tabs)
        return panel

    def _create_overview_tab(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        layout.addWidget(self.dashboard)
        layout.addWidget(self.overview_state)
        return panel

    def _wire_actions(self) -> None:
        c = self.controls
        c.csv_browse.clicked.connect(self._browse_csv)
        c.xml_browse.clicked.connect(self._browse_xml)
        c.csv_load.clicked.connect(self._load_csv_from_ui)
        c.xml_load.clicked.connect(self._load_xml_from_ui)
        c.connect_btn.clicked.connect(self._connect_robot)
        c.init_btn.clicked.connect(self._robot_init)
        c.disable_btn.clicked.connect(self._robot_disable)
        c.set_all_zero_btn.clicked.connect(self._set_zero_all)
        c.kill_backend_btn.clicked.connect(self._kill_remote_backend)
        c.start_backend_btn.clicked.connect(self._start_remote_backend)
        c.apply_mit_btn.clicked.connect(self._apply_mit_params)
        c.start_btn.clicked.connect(self._start)
        c.stop_btn.clicked.connect(self.engine.stop)
        c.prev_btn.clicked.connect(self.engine.prev)
        c.step_btn.clicked.connect(self.engine.step)
        c.frame_spin.valueChanged.connect(self._seek_from_spin)
        c.frame_slider.sliderPressed.connect(self._on_slider_pressed)
        c.frame_slider.sliderReleased.connect(self._seek_from_slider_release)
        c.record_btn.clicked.connect(self._start_recording)
        c.stop_record_btn.clicked.connect(self._stop_recording)
        c.speed_input.editingFinished.connect(self._on_speed_changed)

        self.knee_debug.record_requested.connect(self._start_recording)
        self.knee_debug.stop_record_requested.connect(self._stop_recording)
        self.knee_debug.send_all_requested.connect(self._send_manual_target)
        self.knee_debug.send_single_requested.connect(self._send_manual_target)
        self.knee_debug.set_zero_requested.connect(self._set_zero_joint)

    def _project_root(self) -> Path:
        return Path(__file__).resolve().parent.parent

    def _load_default_files(self) -> None:
        root = self._project_root()
        xml_path = root / 'sim_record' / 'leggedrobot_flat_fixed.xml'
        preferred_csvs = [
            root / 'sim_record' / 'output' / 'recording.csv',
            root / 'recording_20260408_230452.csv',
        ]
        csv_path = next((p for p in preferred_csvs if p.exists()), preferred_csvs[-1])

        self.controls.xml_path.setText(str(xml_path))
        self.controls.csv_path.setText(str(csv_path))
        if xml_path.exists():
            self.engine.load_mujoco(str(xml_path), start_viewer=True)
        if csv_path.exists():
            self.engine.load_csv(str(csv_path))

    def closeEvent(self, event) -> None:  # noqa: N802
        self.engine.close()
        super().closeEvent(event)

    def _browse_csv(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Open replay CSV', str(self._project_root()), 'CSV Files (*.csv)')
        if path:
            self.controls.csv_path.setText(path)

    def _browse_xml(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Open MuJoCo XML', str(self._project_root()), 'XML Files (*.xml)')
        if path:
            self.controls.xml_path.setText(path)

    def _load_csv_from_ui(self) -> None:
        path = self.controls.csv_path.text().strip()
        if path and not self.engine.load_csv(path):
            QMessageBox.warning(self, 'CSV load failed', 'Could not load CSV. See log panel for details.')

    def _load_xml_from_ui(self) -> None:
        path = self.controls.xml_path.text().strip()
        if path and not self.engine.load_mujoco(path, start_viewer=True):
            QMessageBox.warning(self, 'MuJoCo load failed', 'Could not load MuJoCo XML. See log panel for details.')

    def _connect_robot(self) -> None:
        ok = self.engine.connect_robot(
            self.controls.host.text().strip(),
            int(self.controls.cmd_port.text().strip()),
            int(self.controls.state_port.text().strip()),
        )
        if not ok:
            QMessageBox.warning(self, 'Robot connect failed', 'Could not connect to robot. See log panel for details.')

    def _robot_init(self) -> None:
        reply = self.engine.robot_init()
        if not reply.get('ok', False):
            QMessageBox.warning(self, 'Init failed', str(reply))

    def _robot_disable(self) -> None:
        reply = self.engine.robot_disable()
        if not reply.get('ok', False):
            QMessageBox.warning(self, 'Disable failed', str(reply))

    def _apply_mit_params(self) -> None:
        reply = self.engine.robot_set_mit_param(
            self.controls.kp_spin.value(),
            self.controls.kd_spin.value(),
            self.controls.vel_limit_spin.value(),
            self.controls.torque_limit_spin.value(),
        )
        if not reply.get('ok', False):
            QMessageBox.warning(self, 'MIT param failed', str(reply))

    def _set_zero_joint(self, joint_idx: int) -> None:
        reply = self.engine.robot_set_zero_joint(joint_idx)
        if not reply.get('ok', False):
            QMessageBox.warning(self, 'SetZero failed', str(reply))

    def _set_zero_all(self) -> None:
        reply = self.engine.robot_set_zero_all()
        if not reply.get('ok', False):
            QMessageBox.warning(self, 'Set All Zero failed', str(reply))

    def _ssh_host(self) -> str:
        return self.controls.host.text().strip() or '10.20.127.185'

    def _run_remote_backend_job(self, label: str, func) -> None:  # noqa: ANN001
        host = self._ssh_host()
        self.engine.bus.log(f'{label} requested for ares@{host}')

        def worker() -> None:
            reply = func(host)
            status = 'ok' if reply.get('ok', False) else 'failed'
            self.engine.bus.log(f'{label} {status}: {reply.get("msg", "")}'.strip())
            stdout = str(reply.get('stdout', '')).strip()
            stderr = str(reply.get('stderr', '')).strip()
            if stdout:
                self.engine.bus.log(f'{label} stdout: {stdout}')
            if stderr:
                self.engine.bus.log(f'{label} stderr: {stderr}')

        threading.Thread(target=worker, daemon=True).start()

    def _kill_remote_backend(self) -> None:
        self._run_remote_backend_job('SSH kill backend', self.engine.remote_kill_backend)

    def _start_remote_backend(self) -> None:
        self._run_remote_backend_job('SSH start backend', self.engine.remote_start_backend)

    def _parse_speed(self) -> float:
        try:
            return max(0.01, float(self.controls.speed_input.text().strip()))
        except Exception:
            self.controls.speed_input.setText('1.0')
            return 1.0

    def _on_speed_changed(self) -> None:
        speed = self._parse_speed()
        self.engine.set_playback_speed(speed)
        self.controls.speed_input.setText(f'{speed:.2f}'.rstrip('0').rstrip('.'))

    def _start(self) -> None:
        self.engine.start(self.controls.frame_spin.value(), speed=self._parse_speed())

    def _seek_from_spin(self, value: int) -> None:
        snapshot = self.engine.get_snapshot()
        if snapshot.total_frames == 0 or snapshot.cursor == value:
            return
        self.engine.seek(value)

    def _seek_from_slider_release(self) -> None:
        self._slider_pressed = False
        value = self.controls.frame_slider.value()
        snapshot = self.engine.get_snapshot()
        if snapshot.total_frames == 0 or snapshot.cursor == value:
            return
        self.engine.seek(value)

    def _on_slider_pressed(self) -> None:
        self._slider_pressed = True

    def _send_manual_target(self, values: list[float]) -> None:
        target = np.asarray(values, dtype=np.float64)
        if not self.engine.set_manual_target(target):
            QMessageBox.warning(self, 'Send joint failed', 'Failed to send manual target. See logs for details.')

    def _set_recording_state(self, recording: bool) -> None:
        self._recording = recording
        self.controls.record_btn.setEnabled(not recording)
        self.controls.stop_record_btn.setEnabled(recording)
        self.controls.record_btn.setText('Recording...' if recording else 'Record')
        self.knee_debug.set_recording_state(recording)

    def _start_recording(self) -> None:
        if self._recording:
            return
        self._record_start_time = time.monotonic()
        self._record_data.clear()
        self._set_recording_state(True)

    def _stop_recording(self) -> None:
        if not self._recording:
            return
        self._set_recording_state(False)
        if not self._record_data:
            return

        save_dir = self._project_root()
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename = save_dir / f'recording_{timestamp}.csv'
        num_samples = len(self._record_data)

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['time_s']
            for name in JOINT_NAMES:
                header.append(f'target_{name}')
            for name in JOINT_NAMES:
                header.append(f'robot_{name}')
            writer.writerow(header)
            for row in self._record_data:
                writer.writerow(row)

        self._record_data.clear()
        QMessageBox.information(self, 'Recording saved', f'Saved {num_samples} samples to:\n{filename}')

    def _capture_snapshot(self, snapshot) -> None:  # noqa: ANN001
        if not self._recording:
            return
        t = time.monotonic() - self._record_start_time
        target = snapshot.current_target.tolist()
        robot = snapshot.robot_state.positions.tolist() if snapshot.robot_state is not None else [0.0] * 12
        self._record_data.append([t] + target + robot)

    def refresh(self) -> None:
        snapshot = self.engine.get_snapshot()
        self._capture_snapshot(snapshot)

        max_idx = max(0, snapshot.total_frames - 1)
        self.controls.frame_spin.setMaximum(max_idx)
        self.controls.frame_slider.setMaximum(max_idx)
        with QSignalBlocker(self.controls.frame_spin):
            self.controls.frame_spin.setValue(snapshot.cursor)
        with QSignalBlocker(self.controls.frame_slider):
            if not self._slider_pressed:
                self.controls.frame_slider.setValue(snapshot.cursor)
        with QSignalBlocker(self.controls.speed_input):
            self.controls.speed_input.setText(f'{snapshot.playback_speed:.2f}'.rstrip('0').rstrip('.'))

        if snapshot.backend_state is not None:
            if snapshot.backend_state.kp is not None:
                self.controls.kp_spin.setValue(snapshot.backend_state.kp)
            if snapshot.backend_state.kd is not None:
                self.controls.kd_spin.setValue(snapshot.backend_state.kd)

        self.dashboard.update_all(snapshot)
        self.status_panel.update_status(snapshot)
        self.overview_state.update_snapshot(snapshot)
        self.knee_debug.push(snapshot)

        if len(snapshot.log_lines) != self._last_log_count:
            self.logs.setPlainText('\n'.join(snapshot.log_lines))
            self.logs.verticalScrollBar().setValue(self.logs.verticalScrollBar().maximum())
            self._last_log_count = len(snapshot.log_lines)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()
