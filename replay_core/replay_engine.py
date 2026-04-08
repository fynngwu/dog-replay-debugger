from __future__ import annotations

import threading
import time
from typing import Optional

from adapters.mujoco_adapter import MujocoAdapter
from adapters.null_robot_adapter import NullRobotAdapter
from adapters.robot_adapter import RobotAdapter
from replay_core.config import EngineConfig
from replay_core.csv_loader import CsvLoadError, load_replay_csv
from replay_core.sync_bus import SharedStateBus
from replay_core.types import ReplaySequence, RuntimeSnapshot


class ReplayEngine:
    def __init__(self, config: Optional[EngineConfig] = None):
        self.config = config or EngineConfig()
        self.bus = SharedStateBus()
        self.sequence: Optional[ReplaySequence] = None
        self.robot = NullRobotAdapter()
        self.mujoco = MujocoAdapter(self.bus)

        self._play_thread: Optional[threading.Thread] = None
        self._play_stop = threading.Event()
        self._lock = threading.RLock()

    def load_csv(self, path: str) -> bool:
        try:
            sequence = load_replay_csv(path, default_dt=self.config.default_frame_dt)
        except CsvLoadError as exc:
            self.bus.log(f'CSV load failed: {exc}')
            return False
        with self._lock:
            self.sequence = sequence
            self.bus.set_sequence(sequence)
        self.bus.log(
            f'CSV loaded: {sequence.csv_path} ({sequence.total_frames} frames, target={sequence.target_columns}, time={sequence.time_columns}, dt≈{sequence.estimated_dt:.4f}s)'
        )
        return True

    def load_mujoco(self, xml_path: str, start_viewer: bool = True) -> bool:
        ok = self.mujoco.load_model(xml_path, start_viewer=start_viewer)
        if ok and self.sequence is not None:
            self.bus.log('MuJoCo ready and following the local replay cursor')
        return ok

    def connect_robot(self, host: str, cmd_port: Optional[int] = None, state_port: Optional[int] = None) -> bool:
        if isinstance(self.robot, RobotAdapter):
            self.robot.disconnect()
        self.robot = RobotAdapter(self.bus, timeout_s=self.config.robot_timeout_s, tx_period_s=self.config.robot_tx_period_s)
        return self.robot.connect(
            host=host,
            cmd_port=self.config.robot_cmd_port if cmd_port is None else int(cmd_port),
            state_port=self.config.robot_state_port if state_port is None else int(state_port),
        )

    def robot_ping(self) -> bool:
        if not isinstance(self.robot, RobotAdapter):
            self.bus.log('Robot is not connected')
            return False
        return self.robot.ping()

    def robot_init(self, duration_s: float = 2.5):
        if not isinstance(self.robot, RobotAdapter):
            return {'ok': False, 'msg': 'robot is not connected'}
        return self.robot.init(duration_s)

    def robot_enable(self):
        if not isinstance(self.robot, RobotAdapter):
            return {'ok': False, 'msg': 'robot is not connected'}
        return self.robot.enable()

    def robot_disable(self):
        if not isinstance(self.robot, RobotAdapter):
            return {'ok': False, 'msg': 'robot is not connected'}
        return self.robot.disable()

    def start(self, from_idx: Optional[int] = None) -> bool:
        with self._lock:
            if self.sequence is None:
                self.bus.log('Cannot start replay: no CSV loaded')
                return False
            if from_idx is not None:
                self._set_cursor_locked(int(from_idx), playing=False, publish_to_robot=True)
            self.stop()
            self._play_stop.clear()
            self.bus.set_playing(True)
            self._play_thread = threading.Thread(target=self._play_loop, daemon=True)
            self._play_thread.start()
        self.bus.log(f'Replay started from frame {self.bus.current_cursor()}')
        return True

    def stop(self) -> None:
        self._play_stop.set()
        thread = self._play_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._play_thread = None
        self.bus.set_playing(False)

    def seek(self, frame_idx: int) -> bool:
        with self._lock:
            if self.sequence is None:
                self.bus.log('Cannot seek: no CSV loaded')
                return False
            self.stop()
            self._set_cursor_locked(frame_idx, playing=False, publish_to_robot=True)
        self.bus.log(f'Seek -> frame {self.bus.current_cursor()}')
        return True

    def step(self) -> bool:
        with self._lock:
            if self.sequence is None:
                self.bus.log('Cannot step: no CSV loaded')
                return False
            self.stop()
            self._set_cursor_locked(self.bus.current_cursor() + 1, playing=False, publish_to_robot=True)
        return True

    def prev(self) -> bool:
        with self._lock:
            if self.sequence is None:
                self.bus.log('Cannot prev: no CSV loaded')
                return False
            self.stop()
            self._set_cursor_locked(self.bus.current_cursor() - 1, playing=False, publish_to_robot=True)
        return True

    def get_snapshot(self) -> RuntimeSnapshot:
        return self.bus.snapshot()

    def close(self) -> None:
        self.stop()
        if isinstance(self.robot, RobotAdapter):
            self.robot.disconnect()
        self.mujoco.close()

    def _set_cursor_locked(self, frame_idx: int, playing: bool, publish_to_robot: bool) -> None:
        assert self.sequence is not None
        frame_idx = max(0, min(int(frame_idx), self.sequence.total_frames - 1))
        frame = self.sequence.frames[frame_idx]
        self.bus.set_cursor_target(frame.index, frame.target_rel, playing=playing, publish_to_robot=publish_to_robot)

    def _play_loop(self) -> None:
        with self._lock:
            sequence = self.sequence
            if sequence is None:
                return
            start_cursor = self.bus.current_cursor()
            start_time_sec = sequence.frames[start_cursor].time_sec
        start_wall = time.monotonic()
        idx = start_cursor
        while not self._play_stop.is_set() and sequence is not None and idx < sequence.total_frames:
            elapsed = time.monotonic() - start_wall
            replay_time = start_time_sec + elapsed
            while idx + 1 < sequence.total_frames and sequence.frames[idx + 1].time_sec <= replay_time:
                idx += 1
            with self._lock:
                if self.sequence is None:
                    break
                frame = self.sequence.frames[idx]
                self.bus.set_cursor_target(frame.index, frame.target_rel, playing=True, publish_to_robot=True)
            if idx >= sequence.total_frames - 1:
                break
            time.sleep(self.config.play_loop_sleep_s)
        with self._lock:
            if self.sequence is not None:
                final_idx = min(idx, self.sequence.total_frames - 1)
                frame = self.sequence.frames[final_idx]
                self.bus.set_cursor_target(frame.index, frame.target_rel, playing=False, publish_to_robot=True)
        self.bus.set_playing(False)
        if sequence is not None:
            self.bus.log(f'Replay stopped at frame {min(idx, sequence.total_frames - 1)}')
