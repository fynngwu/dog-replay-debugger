from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import threading
import time

from PySide6.QtCore import QSignalBlocker, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from client import SMClient
from control_panel import ControlPanel
from status_panel import StatusPanel
from styles import STYLE

from replay_core.csv_loader import CsvLoadError, load_replay_csv
from replay_core.joint_limits import clamp_relative_targets
from replay_core.sync_bus import SharedStateBus
from replay_core.types import ReplaySequence

from adapters.mujoco_adapter import MujocoAdapter

DEFAULT_CSV_PATH = str(
    Path(__file__).resolve().parents[2] / "sim_record" / "output" / "recording.csv"
)
MUJOCO_XML_PATH = str(
    Path(__file__).resolve().parents[2] / "sim_record" / "leggedrobot_flat_fixed.xml"
)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dog State Machine Controller")
        self.resize(1200, 800)
        self.setStyleSheet(STYLE)

        self.client = SMClient()
        self.controls = ControlPanel()
        self.status = StatusPanel()

        self.bus = SharedStateBus()
        self.mujoco = MujocoAdapter(self.bus)

        self._replaying = False
        self._replay_error: str | None = None
        self._replay_thread: threading.Thread | None = None
        self._sequence: ReplaySequence | None = None
        self._current_frame: int = 0

        self._setup_ui()
        self._wire_actions()
        self.controls.csv_path.setText(DEFAULT_CSV_PATH)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(20)

    def _setup_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(12)

        splitter = QSplitter()
        splitter.setHandleWidth(2)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(self.controls)
        splitter.addWidget(left)

        splitter.addWidget(self.status)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([420, 780])
        layout.addWidget(splitter)

    def _wire_actions(self) -> None:
        c = self.controls
        c.connect_btn.clicked.connect(self._connect)
        c.disconnect_btn.clicked.connect(self._disconnect)
        c.init_btn.clicked.connect(lambda: self._request_mode("init"))
        c.execute_btn.clicked.connect(lambda: self._request_mode("execute"))
        c.policy_btn.clicked.connect(lambda: self._request_mode("policy"))
        c.stop_btn.clicked.connect(lambda: self._request_mode("stop"))
        c.send_target_btn.clicked.connect(self._send_target)
        c.csv_browse.clicked.connect(self._browse_csv)
        c.replay_start_btn.clicked.connect(self._start_replay)
        c.replay_stop_btn.clicked.connect(self._stop_replay)
        c.mujoco_load_btn.clicked.connect(self._load_mujoco)
        c.mujoco_close_btn.clicked.connect(self._close_mujoco)
        self.controls.target_changed.connect(self._on_target_changed)

    def _log(self, msg: str) -> None:
        self.status.append_log(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.bus.log(msg)

    def _connect(self) -> None:
        host = self.controls.host_input.text().strip() or "127.0.0.1"
        try:
            port = int(self.controls.port_input.text().strip())
        except ValueError:
            port = 48001
        self.client = SMClient(host, port)
        ok, msg = self.client.connect()
        if ok:
            self._log(f"connected to {host}:{port}")
            self.controls.connect_btn.setEnabled(False)
        else:
            self._log(f"connect failed: {msg}")
            QMessageBox.warning(self, "Connect failed", msg)

    def _disconnect(self) -> None:
        self._stop_replay()
        self.client.disconnect()
        self._log("disconnected")
        self.controls.connect_btn.setEnabled(True)
        self.status.update_mode("---")

    def _request_mode(self, mode: str) -> None:
        self._log(f"request_mode {mode}")
        reply = self.client.request_mode(mode)
        if reply.get("ok"):
            self._log(reply.get("msg", ""))
        else:
            self._log(f"error: {reply.get('msg', '')}")
            QMessageBox.warning(self, "Mode error", reply.get("msg", "unknown error"))

    def _send_target(self) -> None:
        joints = self.controls.get_target_values()
        self._log("sending target...")
        reply = self.client.send_target(joints)
        if reply.get("ok"):
            self._log(reply.get("msg", ""))
        else:
            self._log(f"error: {reply.get('msg', '')}")

    def _browse_csv(self) -> None:
        path, _ = __import__(
            "PySide6.QtWidgets", fromlist=["QFileDialog"]
        ).QFileDialog.getOpenFileName(self, "Open CSV", "", "CSV Files (*.csv)")
        if path:
            self.controls.csv_path.setText(path)

    def _load_csv(self) -> bool:
        path = self.controls.csv_path.text().strip()
        if not path:
            return False
        try:
            sequence = load_replay_csv(path)
        except CsvLoadError as exc:
            QMessageBox.warning(self, "CSV error", str(exc))
            return False
        self._sequence = sequence
        self.bus.set_sequence(sequence)
        self.controls.frame_spin.setMaximum(sequence.total_frames - 1)
        self.controls.frame_spin.setValue(0)
        self._log(f"CSV loaded: {sequence.csv_path} ({sequence.total_frames} frames)")
        return True

    def _start_replay(self) -> None:
        path = self.controls.csv_path.text().strip()
        if not path:
            QMessageBox.warning(self, "No CSV", "Please specify a CSV path")
            return
        if self._sequence is None:
            if not self._load_csv():
                return

        self._log(f"starting replay: {self._sequence.total_frames} frames, speed={self.controls.speed_spin.value()}x")
        self._replaying = True
        self._replay_error = None
        self._current_frame = 0
        speed = self.controls.speed_spin.value()
        self.controls.replay_start_btn.setEnabled(False)
        self.controls.replay_stop_btn.setEnabled(True)
        t = threading.Thread(target=self._replay_worker, args=(speed,), daemon=True)
        t.start()
        self._replay_thread = t

    def _stop_replay(self) -> None:
        if not self._replaying:
            return
        self._replaying = False

    def _replay_worker(self, speed: float) -> None:
        """Run in background thread. Must NOT touch any Qt widget."""
        try:
            frames = self._sequence.frames
            last_wall = time.monotonic()
            for i, frame in enumerate(frames):
                if not self._replaying:
                    break
                # Send target joints via TCP (non-fatal if not connected)
                reply = self.client.send_target(frame.target_rel.tolist())
                if not reply.get("ok") and self.client.is_connected:
                    # TCP error while connected — real problem
                    self._replay_error = reply.get("msg", "unknown")
                    self._current_frame = i
                    break
                # Push to bus for MuJoCo viewer
                self.bus.set_cursor_target(
                    frame.index, frame.target_rel, frame.target_rel,
                    playing=True, publish_to_robot=False,
                )
                self._current_frame = frame.index
                # Pace playback using CSV timestamps
                if i > 0:
                    dt = frame.time_sec - frames[i - 1].time_sec
                    elapsed = time.monotonic() - last_wall
                    if elapsed < dt / speed:
                        time.sleep(dt / speed - elapsed)
                last_wall = time.monotonic()
        except Exception as e:
            self._replay_error = f"worker exception: {e}"
        finally:
            self._replaying = False

    def _load_mujoco(self) -> None:
        ok = self.mujoco.load_model(MUJOCO_XML_PATH)
        if ok:
            self._log(f"MuJoCo loaded: {MUJOCO_XML_PATH}")
            self.controls.mujoco_load_btn.setEnabled(False)
            self.controls.mujoco_close_btn.setEnabled(True)
        else:
            self._log("MuJoCo load failed")
            QMessageBox.warning(self, "MuJoCo", "Failed to load MuJoCo model")

    def _close_mujoco(self) -> None:
        self.mujoco.close()
        self._log("MuJoCo closed")
        self.controls.mujoco_load_btn.setEnabled(True)
        self.controls.mujoco_close_btn.setEnabled(False)

    def _on_target_changed(self, target: list[float]) -> None:
        snapshot = self.bus.snapshot()
        mujoco_pos = snapshot.mujoco_state.positions.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
        mujoco_vel = snapshot.mujoco_state.velocities.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
        robot_pos = [0.0] * 12
        robot_vel = [0.0] * 12
        self.status.update_joints(target, robot_pos, robot_vel, mujoco_pos, mujoco_vel)

    def _refresh(self) -> None:
        # Sync bus logs to GUI log panel (always, even during replay)
        snapshot = self.bus.snapshot()
        for line in snapshot.log_lines:
            if not self.status.log_edit.toPlainText().endswith(line):
                self.status.append_log(line)

        # Detect replay thread death (must check BEFORE the replaying guard)
        if self._replay_thread is not None and not self._replay_thread.is_alive():
            if self._replay_error:
                self._log(f"replay error: {self._replay_error}")
            else:
                self._log("replay finished")
            self.controls.replay_start_btn.setEnabled(True)
            self.controls.replay_stop_btn.setEnabled(False)
            self._replay_thread = None

        # Update frame spin from worker thread state (main-thread-safe)
        if self._replaying:
            with QSignalBlocker(self.controls.frame_spin):
                self.controls.frame_spin.setValue(self._current_frame)
            # Still refresh MuJoCo display during replay (skip TCP polling)
            target = snapshot.current_target.tolist()
            mujoco_pos = snapshot.mujoco_state.positions.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
            mujoco_vel = snapshot.mujoco_state.velocities.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
            mujoco_gyro = snapshot.mujoco_imu[:3].tolist() if snapshot.mujoco_imu is not None else [0.0, 0.0, 0.0]
            mujoco_gravity = snapshot.mujoco_imu[3:].tolist() if snapshot.mujoco_imu is not None else [0.0, 0.0, 0.0]
            self.status.update_joints(target, [0.0] * 12, [0.0] * 12, mujoco_pos, mujoco_vel)
            self.status.update_imu([0.0] * 3, [0.0] * 3, mujoco_gyro, mujoco_gravity)
            return

        if not self.client.is_connected:
            return

        reply = self.client.get_all()
        if not reply.get("ok"):
            return

        data = reply["data"]
        self.status.update_mode(data.get("mode", "---"))

        robot_pos = [j["position"] for j in data.get("joints", [])] or [0.0] * 12
        robot_vel = [j["velocity"] for j in data.get("joints", [])] or [0.0] * 12
        imu_data = data.get("imu", {})
        robot_gyro = [imu_data.get("gyro_x", 0.0), imu_data.get("gyro_y", 0.0), imu_data.get("gyro_z", 0.0)]
        robot_gravity = [imu_data.get("grav_x", 0.0), imu_data.get("grav_y", 0.0), imu_data.get("grav_z", 0.0)]

        target = snapshot.current_target.tolist()
        mujoco_pos = snapshot.mujoco_state.positions.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
        mujoco_vel = snapshot.mujoco_state.velocities.tolist() if snapshot.mujoco_state is not None else [0.0] * 12
        mujoco_gyro = snapshot.mujoco_imu[:3].tolist() if snapshot.mujoco_imu is not None else [0.0, 0.0, 0.0]
        mujoco_gravity = snapshot.mujoco_imu[3:].tolist() if snapshot.mujoco_imu is not None else [0.0, 0.0, 0.0]

        self.status.update_joints(target, robot_pos, robot_vel, mujoco_pos, mujoco_vel)
        self.status.update_imu(robot_gyro, robot_gravity, mujoco_gyro, mujoco_gravity)

    def closeEvent(self, event) -> None:
        self._stop_replay()
        self.mujoco.close()
        self.client.disconnect()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
