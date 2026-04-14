from __future__ import annotations

import sys
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
from status_panel import StatusPanel, load_csv_frames
from styles import STYLE


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dog State Machine Controller")
        self.resize(1200, 800)
        self.setStyleSheet(STYLE)

        self.client = SMClient()
        self.controls = ControlPanel()
        self.status = StatusPanel()

        self._replaying = False
        self._replay_thread: threading.Thread | None = None
        self._csv_frames: list[list[float]] = []
        self._start_time: float = 0.0

        self._setup_ui()
        self._wire_actions()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(100)

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

    def _log(self, msg: str) -> None:
        self.status.append_log(f"[{time.strftime('%H:%M:%S')}] {msg}")

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
        joints = [spin.value() for spin in self.controls.target_spins]
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

    def _start_replay(self) -> None:
        path = self.controls.csv_path.text().strip()
        if not path:
            QMessageBox.warning(self, "No CSV", "Please specify a CSV path")
            return
        ok, msg, frames = load_csv_frames(path)
        if not ok:
            QMessageBox.warning(self, "CSV error", msg)
            return
        if not frames:
            QMessageBox.warning(self, "CSV empty", "No valid frames found")
            return

        self._csv_frames = frames
        self.controls.frame_spin.setMaximum(len(frames) - 1)
        self.controls.frame_spin.setValue(0)
        self._replaying = True
        self.controls.replay_start_btn.setEnabled(False)
        self.controls.replay_stop_btn.setEnabled(True)
        self._log(f"replay started: {len(frames)} frames")
        self._start_time = time.monotonic()
        self._replay_thread = threading.Thread(target=self._replay_worker, daemon=True)
        self._replay_thread.start()

    def _stop_replay(self) -> None:
        if not self._replaying:
            return
        self._replaying = False
        self.controls.replay_start_btn.setEnabled(True)
        self.controls.replay_stop_btn.setEnabled(False)
        self._log("replay stopped")

    def _replay_worker(self) -> None:
        interval = self.controls.interval_spin.value()
        for i, frame in enumerate(self._csv_frames):
            if not self._replaying:
                break
            reply = self.client.send_target(frame)
            if not reply.get("ok"):
                self._log(f"replay error frame {i}: {reply.get('msg', '')}")
                break
            time.sleep(interval)
        self._replaying = False
        self.controls.replay_start_btn.setEnabled(True)
        self.controls.replay_stop_btn.setEnabled(False)

    def _refresh(self) -> None:
        if not self.client.is_connected:
            return

        mode_reply = self.client.get_mode()
        if mode_reply.get("ok"):
            mode_str = mode_reply.get("msg", "").replace("current mode: ", "")
            self.status.update_mode(mode_str)

        joints_reply = self.client.get_joints()
        if joints_reply.get("ok"):
            data = joints_reply.get("data", {})
            self.status.update_joints(
                data.get("position", [0] * 12), data.get("velocity", [0] * 12)
            )

        imu_reply = self.client.get_imu()
        if imu_reply.get("ok"):
            data = imu_reply.get("data", {})
            self.status.update_imu(
                data.get("gyro", [0] * 3), data.get("gravity", [0] * 3)
            )

        if self._replaying:
            elapsed = time.monotonic() - self._start_time
            with QSignalBlocker(self.controls.frame_spin):
                frame_idx = int(
                    elapsed / max(self.controls.interval_spin.value(), 0.001)
                )
                self.controls.frame_spin.setValue(
                    min(frame_idx, len(self._csv_frames) - 1)
                )

    def closeEvent(self, event) -> None:
        self._stop_replay()
        self.client.disconnect()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
