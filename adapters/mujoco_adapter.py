from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from replay_core.constants import NUM_JOINTS, POLICY_TO_SIM, SIM_TO_POLICY
from replay_core.metrics import RateTracker
from replay_core.sync_bus import SharedStateBus

try:
    import mujoco
    import mujoco.viewer
except Exception:  # pragma: no cover - optional in non-GUI test environments
    mujoco = None

KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17.0, 17.0, 25.0] * 4, dtype=np.float64)


class MujocoAdapter:
    def __init__(self, bus: SharedStateBus):
        self.bus = bus
        self.model = None
        self.data = None
        self.xml_path: Optional[Path] = None
        self.default_joint_sim = np.zeros(NUM_JOINTS, dtype=np.float64)
        self.target_q_sim = np.zeros(NUM_JOINTS, dtype=np.float64)

        self._has_freejoint: bool = False
        self._qpos_offset: int = 0
        self._qvel_offset: int = 0

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._sim_thread: Optional[threading.Thread] = None
        self._viewer_thread: Optional[threading.Thread] = None
        self._viewer_running = False
        self._apply_rate = RateTracker()

    @property
    def loaded(self) -> bool:
        return self.model is not None and self.data is not None

    @property
    def viewer_running(self) -> bool:
        return self._viewer_running

    def load_model(self, xml_path: str, start_viewer: bool = True) -> bool:
        if mujoco is None:
            self.bus.log('MuJoCo is not installed in this Python environment')
            return False
        self.close()
        resolved = Path(xml_path).expanduser().resolve()
        try:
            self.model = mujoco.MjModel.from_xml_path(str(resolved))
            self.model.opt.timestep = 0.002
            self.data = mujoco.MjData(self.model)
            # Detect whether the model has a freejoint (floating base)
            # via the xml filename: files containing "_fixed" are fixed-base
            self._has_freejoint = "_fixed" not in resolved.stem
            if self._has_freejoint:
                self._qpos_offset = 7  # 3 pos + 4 quat
                self._qvel_offset = 6  # 3 linear + 3 angular
            else:
                self._qpos_offset = 0
                self._qvel_offset = 0
            self.default_joint_sim = np.array(
                self.model.qpos0[self._qpos_offset:self._qpos_offset + NUM_JOINTS], dtype=np.float64
            )
            self.target_q_sim = self.default_joint_sim.copy()
            self.xml_path = resolved
            self._stop.clear()
            self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
            self._sim_thread.start()
            if start_viewer:
                self._viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
                self._viewer_thread.start()
            self.bus.set_mujoco_status(True, start_viewer)
            self.bus.log(f'MuJoCo model loaded: {resolved}')
            return True
        except Exception as exc:
            self.bus.log(f'MuJoCo load failed: {exc}')
            self.model = None
            self.data = None
            self.bus.set_mujoco_status(False, False)
            return False

    def close(self) -> None:
        self._stop.set()
        for thread in (self._viewer_thread, self._sim_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
        self._viewer_thread = None
        self._sim_thread = None
        self._viewer_running = False
        self.model = None
        self.data = None
        self.bus.set_mujoco_status(False, False)
        self.bus.set_mujoco_apply_hz(0.0)

    def _sim_loop(self) -> None:
        if mujoco is None or self.model is None or self.data is None:
            return
        emit_every = 0.02
        next_emit = time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                qpos_off = self._qpos_offset
                qvel_off = self._qvel_offset
                target_rel = self.bus.get_target()
                self.target_q_sim = target_rel[POLICY_TO_SIM] + self.default_joint_sim
                q_sim = np.array(self.data.qpos[qpos_off:qpos_off + NUM_JOINTS], dtype=np.float64)
                dq_sim = np.array(self.data.qvel[qvel_off:qvel_off + NUM_JOINTS], dtype=np.float64)
                tau = (self.target_q_sim - q_sim) * KP - dq_sim * KD
                tau = np.clip(tau, -TAU_LIMIT, TAU_LIMIT)
                self.data.ctrl[:] = tau
                mujoco.mj_step(self.model, self.data)
                self._apply_rate.tick()
                self.bus.set_mujoco_apply_hz(self._apply_rate.value())

                now = time.monotonic()
                if now >= next_emit:
                    q_sim = np.array(self.data.qpos[qpos_off:qpos_off + NUM_JOINTS], dtype=np.float64)
                    dq_sim = np.array(self.data.qvel[qvel_off:qvel_off + NUM_JOINTS], dtype=np.float64)
                    q_policy_rel = q_sim[SIM_TO_POLICY] - self.default_joint_sim[SIM_TO_POLICY]
                    dq_policy = dq_sim[SIM_TO_POLICY]
                    self.bus.update_mujoco_state(q_policy_rel, dq_policy, now)
                    next_emit = now + emit_every
            time.sleep(float(self.model.opt.timestep))

    def _viewer_loop(self) -> None:
        if mujoco is None or self.model is None or self.data is None:
            return
        try:
            self._viewer_running = True
            self.bus.set_mujoco_status(True, True)
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                viewer.cam.distance = 3.0
                viewer.cam.azimuth = 90.0
                viewer.cam.elevation = -45.0
                while viewer.is_running() and not self._stop.is_set():
                    with self._lock:
                        viewer.sync()
                    time.sleep(0.01)
        except Exception as exc:
            self.bus.log(f'MuJoCo viewer error: {exc}')
        finally:
            self._viewer_running = False
            self.bus.set_mujoco_status(self.loaded, False)
