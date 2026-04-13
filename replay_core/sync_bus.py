from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional

import numpy as np

from replay_core.constants import NUM_JOINTS
from replay_core.types import BackendState, JointState, ReplaySequence, RuntimeSnapshot


class SharedStateBus:
    def __init__(self):
        self._lock = threading.RLock()
        self._sequence: Optional[ReplaySequence] = None
        self._cursor = 0
        self._playing = False
        self._playback_speed = 1.0
        self._target_raw = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._target = np.zeros(NUM_JOINTS, dtype=np.float64)
        self._display_command_seq = 0
        self._robot_command_seq = 0
        self._robot_connected = False
        self._robot_tx_hz = 0.0
        self._robot_rx_hz = 0.0
        self._robot_state: Optional[JointState] = None
        self._backend_state: Optional[BackendState] = None
        self._mujoco_loaded = False
        self._mujoco_viewer_running = False
        self._mujoco_apply_hz = 0.0
        self._mujoco_state: Optional[JointState] = None
        self._logs: Deque[str] = deque(maxlen=200)

    def log(self, message: str) -> None:
        timestamp = time.strftime('%H:%M:%S')
        with self._lock:
            self._logs.append(f'[{timestamp}] {message}')

    def set_sequence(self, sequence: ReplaySequence) -> None:
        with self._lock:
            self._sequence = sequence
            self._cursor = 0
            self._playing = False
            self._playback_speed = 1.0
            self._target_raw = sequence.frames[0].target_rel.copy()
            self._target = sequence.frames[0].target_rel.copy()
            self._display_command_seq += 1

    def clear_sequence(self) -> None:
        with self._lock:
            self._sequence = None
            self._cursor = 0
            self._playing = False
            self._playback_speed = 1.0
            self._target_raw = np.zeros(NUM_JOINTS, dtype=np.float64)
            self._target = np.zeros(NUM_JOINTS, dtype=np.float64)
            self._display_command_seq += 1

    def set_cursor_target(
        self,
        cursor: int,
        raw_target: np.ndarray,
        limited_target: np.ndarray,
        playing: bool,
        publish_to_robot: bool,
    ) -> None:
        with self._lock:
            self._cursor = int(cursor)
            self._target_raw = np.array(raw_target, dtype=np.float64)
            self._target = np.array(limited_target, dtype=np.float64)
            self._playing = bool(playing)
            self._display_command_seq += 1
            if publish_to_robot:
                self._robot_command_seq += 1

    def set_playing(self, playing: bool) -> None:
        with self._lock:
            self._playing = bool(playing)

    def set_playback_speed(self, speed: float) -> None:
        with self._lock:
            self._playback_speed = float(speed)

    def get_target(self) -> np.ndarray:
        with self._lock:
            return self._target.copy()

    def get_raw_target(self) -> np.ndarray:
        with self._lock:
            return self._target_raw.copy()

    def get_robot_command_seq(self) -> int:
        with self._lock:
            return self._robot_command_seq

    def get_display_command_seq(self) -> int:
        with self._lock:
            return self._display_command_seq

    def current_cursor(self) -> int:
        with self._lock:
            return self._cursor

    def is_playing(self) -> bool:
        with self._lock:
            return self._playing

    def get_sequence(self) -> Optional[ReplaySequence]:
        with self._lock:
            return self._sequence

    def set_robot_connected(self, connected: bool) -> None:
        with self._lock:
            self._robot_connected = bool(connected)

    def set_robot_rates(self, tx_hz: float, rx_hz: float) -> None:
        with self._lock:
            self._robot_tx_hz = float(tx_hz)
            self._robot_rx_hz = float(rx_hz)

    def update_robot_state(self, positions: np.ndarray, velocities: np.ndarray, torques: np.ndarray, timestamp_sec: float) -> None:
        with self._lock:
            self._robot_state = JointState(
                positions=np.array(positions, dtype=np.float64),
                velocities=np.array(velocities, dtype=np.float64),
                torques=np.array(torques, dtype=np.float64),
                timestamp_sec=float(timestamp_sec),
            )

    def update_backend_state(self, backend_state: BackendState) -> None:
        with self._lock:
            self._backend_state = BackendState(
                ok=bool(backend_state.ok),
                enabled=bool(backend_state.enabled),
                worker_started=bool(backend_state.worker_started),
                busy=bool(backend_state.busy),
                init_in_progress=bool(backend_state.init_in_progress),
                queue_size=int(backend_state.queue_size),
                kp=None if backend_state.kp is None else float(backend_state.kp),
                kd=None if backend_state.kd is None else float(backend_state.kd),
                joint_positions=np.array(backend_state.joint_positions, dtype=np.float64),
                joint_torques=np.array(backend_state.joint_torques, dtype=np.float64),
                target_joint_positions=np.array(backend_state.target_joint_positions, dtype=np.float64),
                last_sent_joint_positions=np.array(backend_state.last_sent_joint_positions, dtype=np.float64),
                last_error=str(backend_state.last_error),
                raw_json=str(backend_state.raw_json),
                timestamp_sec=float(backend_state.timestamp_sec),
            )

    def set_mujoco_status(self, loaded: bool, viewer_running: bool) -> None:
        with self._lock:
            self._mujoco_loaded = bool(loaded)
            self._mujoco_viewer_running = bool(viewer_running)

    def set_mujoco_apply_hz(self, apply_hz: float) -> None:
        with self._lock:
            self._mujoco_apply_hz = float(apply_hz)

    def update_mujoco_state(self, positions: np.ndarray, velocities: np.ndarray, timestamp_sec: float) -> None:
        with self._lock:
            self._mujoco_state = JointState(
                positions=np.array(positions, dtype=np.float64),
                velocities=np.array(velocities, dtype=np.float64),
                torques=np.zeros(NUM_JOINTS, dtype=np.float64),
                timestamp_sec=float(timestamp_sec),
            )

    def snapshot(self) -> RuntimeSnapshot:
        now = time.monotonic()
        with self._lock:
            csv_path = None if self._sequence is None else str(self._sequence.csv_path)
            total_frames = 0 if self._sequence is None else self._sequence.total_frames
            robot_state_age = None if self._robot_state is None else max(0.0, now - self._robot_state.timestamp_sec)
            backend_state_age = None if self._backend_state is None else max(0.0, now - self._backend_state.timestamp_sec)
            mujoco_state_age = None if self._mujoco_state is None else max(0.0, now - self._mujoco_state.timestamp_sec)
            backend_state = None
            if self._backend_state is not None:
                backend_state = BackendState(
                    ok=bool(self._backend_state.ok),
                    enabled=bool(self._backend_state.enabled),
                    worker_started=bool(self._backend_state.worker_started),
                    busy=bool(self._backend_state.busy),
                    init_in_progress=bool(self._backend_state.init_in_progress),
                    queue_size=int(self._backend_state.queue_size),
                    kp=None if self._backend_state.kp is None else float(self._backend_state.kp),
                    kd=None if self._backend_state.kd is None else float(self._backend_state.kd),
                    joint_positions=self._backend_state.joint_positions.copy(),
                    joint_torques=self._backend_state.joint_torques.copy(),
                    target_joint_positions=self._backend_state.target_joint_positions.copy(),
                    last_sent_joint_positions=self._backend_state.last_sent_joint_positions.copy(),
                    last_error=str(self._backend_state.last_error),
                    raw_json=str(self._backend_state.raw_json),
                    timestamp_sec=float(self._backend_state.timestamp_sec),
                )
            return RuntimeSnapshot(
                csv_path=csv_path,
                sequence_loaded=self._sequence is not None,
                total_frames=total_frames,
                cursor=self._cursor,
                playing=self._playing,
                playback_speed=self._playback_speed,
                current_target_raw=self._target_raw.copy(),
                current_target=self._target.copy(),
                robot_connected=self._robot_connected,
                robot_tx_hz=self._robot_tx_hz,
                robot_rx_hz=self._robot_rx_hz,
                robot_state=None if self._robot_state is None else JointState(self._robot_state.positions.copy(), self._robot_state.velocities.copy(), self._robot_state.torques.copy(), self._robot_state.timestamp_sec),
                robot_state_age_s=robot_state_age,
                backend_state=backend_state,
                backend_state_age_s=backend_state_age,
                mujoco_loaded=self._mujoco_loaded,
                mujoco_viewer_running=self._mujoco_viewer_running,
                mujoco_apply_hz=self._mujoco_apply_hz,
                mujoco_state=None if self._mujoco_state is None else JointState(self._mujoco_state.positions.copy(), self._mujoco_state.velocities.copy(), self._mujoco_state.torques.copy(), self._mujoco_state.timestamp_sec),
                mujoco_state_age_s=mujoco_state_age,
                log_lines=list(self._logs),
            )
