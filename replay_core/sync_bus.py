from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional

import numpy as np

from replay_core.constants import NUM_JOINTS
from replay_core.types import JointState, ReplaySequence, RuntimeSnapshot


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

    def update_robot_state(self, positions: np.ndarray, velocities: np.ndarray, timestamp_sec: float) -> None:
        with self._lock:
            self._robot_state = JointState(
                positions=np.array(positions, dtype=np.float64),
                velocities=np.array(velocities, dtype=np.float64),
                timestamp_sec=float(timestamp_sec),
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
                timestamp_sec=float(timestamp_sec),
            )

    def snapshot(self) -> RuntimeSnapshot:
        now = time.monotonic()
        with self._lock:
            csv_path = None if self._sequence is None else str(self._sequence.csv_path)
            total_frames = 0 if self._sequence is None else self._sequence.total_frames
            robot_state_age = None if self._robot_state is None else max(0.0, now - self._robot_state.timestamp_sec)
            mujoco_state_age = None if self._mujoco_state is None else max(0.0, now - self._mujoco_state.timestamp_sec)
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
                robot_state=None if self._robot_state is None else JointState(self._robot_state.positions.copy(), self._robot_state.velocities.copy(), self._robot_state.timestamp_sec),
                robot_state_age_s=robot_state_age,
                mujoco_loaded=self._mujoco_loaded,
                mujoco_viewer_running=self._mujoco_viewer_running,
                mujoco_apply_hz=self._mujoco_apply_hz,
                mujoco_state=None if self._mujoco_state is None else JointState(self._mujoco_state.positions.copy(), self._mujoco_state.velocities.copy(), self._mujoco_state.timestamp_sec),
                mujoco_state_age_s=mujoco_state_age,
                log_lines=list(self._logs),
            )
