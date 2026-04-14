from __future__ import annotations

import multiprocessing as mp
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from replay_core.constants import NUM_JOINTS, POLICY_TO_SIM, SIM_TO_POLICY
from replay_core.sync_bus import SharedStateBus

try:
    import mujoco
    import mujoco.viewer
except Exception:
    mujoco = None

KP = 25.0
KD = 0.5
TAU_LIMIT = np.array([17.0, 17.0, 25.0] * 4, dtype=np.float64)


def _quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo quaternion from wxyz to xyzw order."""
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)


def _quat_rotate_inverse_xyzw(quat_xyzw: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate a vector by the inverse of a unit quaternion in xyzw order."""
    x, y, z, w = quat_xyzw
    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    return rot.T @ vec


def _mujoco_subprocess(
    xml_path: str,
    target_buf: mp.RawArray,
    state_buf: mp.RawArray,
    imu_buf: mp.RawArray,
    stop_event: mp.Event,
    policy_to_sim: list[int],
    sim_to_policy: list[int],
) -> None:
    """MuJoCo sim + viewer in an isolated subprocess to avoid Qt/OpenGL conflict."""
    if mujoco is None:
        return

    model = mujoco.MjModel.from_xml_path(xml_path)
    model.opt.timestep = 0.002
    data = mujoco.MjData(model)

    has_freejoint = "_fixed" not in Path(xml_path).stem
    qpos_off = 7 if has_freejoint else 0
    qvel_off = 6 if has_freejoint else 0

    default_sim = np.array(
        model.qpos0[qpos_off:qpos_off + NUM_JOINTS], dtype=np.float64
    )

    target_np = np.frombuffer(target_buf, dtype=np.float64)
    state_np = np.frombuffer(state_buf, dtype=np.float64)
    imu_np = np.frombuffer(imu_buf, dtype=np.float64)
    p2s = np.array(policy_to_sim)
    s2p = np.array(sim_to_policy)
    gyro_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -45.0

        last_state_t = time.monotonic()
        while viewer.is_running() and not stop_event.is_set():
            target_q_sim = target_np[p2s] + default_sim

            q = data.qpos[qpos_off:qpos_off + NUM_JOINTS]
            dq = data.qvel[qvel_off:qvel_off + NUM_JOINTS]
            tau = np.clip((target_q_sim - q) * KP - dq * KD, -TAU_LIMIT, TAU_LIMIT)
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)

            now = time.monotonic()
            if now - last_state_t >= 0.02:
                q = data.qpos[qpos_off:qpos_off + NUM_JOINTS]
                dq = data.qvel[qvel_off:qvel_off + NUM_JOINTS]
                state_np[:NUM_JOINTS] = q[s2p] - default_sim[s2p]
                state_np[NUM_JOINTS:] = dq[s2p]
                # IMU: gyro from sensor, projected gravity from orientation
                if gyro_sensor_id != -1:
                    adr = model.sensor_adr[gyro_sensor_id]
                    imu_np[:3] = data.sensordata[adr:adr + 3]
                else:
                    imu_np[:3] = dq[qvel_off:qvel_off + 3]
                if has_freejoint:
                    quat_wxyz = np.array(data.qpos[3:7], dtype=np.float64)
                    quat_xyzw = _quat_wxyz_to_xyzw(quat_wxyz)
                    imu_np[3:] = _quat_rotate_inverse_xyzw(quat_xyzw, np.array([0.0, 0.0, -1.0]))
                last_state_t = now

            viewer.sync()
            time.sleep(float(model.opt.timestep))


class MujocoAdapter:
    def __init__(self, bus: SharedStateBus):
        self.bus = bus
        self.xml_path: Optional[Path] = None
        self._target_buf: Optional[mp.RawArray] = None
        self._state_buf: Optional[mp.RawArray] = None
        self._imu_buf: Optional[mp.RawArray] = None
        self._stop_event: Optional[mp.Event] = None
        self._process: Optional[mp.Process] = None
        self._bridge_thread: Optional[threading.Thread] = None

    @property
    def loaded(self) -> bool:
        return self._process is not None and self._process.is_alive()

    @property
    def viewer_running(self) -> bool:
        return self.loaded

    def load_model(self, xml_path: str, start_viewer: bool = True) -> bool:
        if mujoco is None:
            self.bus.log("MuJoCo is not installed")
            return False
        self.close()
        try:
            resolved = Path(xml_path).expanduser().resolve()
            self.xml_path = resolved

            self._target_buf = mp.RawArray("d", NUM_JOINTS)
            self._state_buf = mp.RawArray("d", NUM_JOINTS * 2)
            self._imu_buf = mp.RawArray("d", 6)
            self._stop_event = mp.Event()

            self._process = mp.Process(
                target=_mujoco_subprocess,
                args=(
                    str(resolved),
                    self._target_buf,
                    self._state_buf,
                    self._imu_buf,
                    self._stop_event,
                    POLICY_TO_SIM.tolist(),
                    SIM_TO_POLICY.tolist(),
                ),
                daemon=True,
            )
            self._process.start()

            self._bridge_thread = threading.Thread(
                target=self._bridge_loop, daemon=True
            )
            self._bridge_thread.start()

            self.bus.set_mujoco_status(True, start_viewer)
            self.bus.log(f"MuJoCo model loaded: {resolved}")
            return True
        except Exception as exc:
            self.bus.log(f"MuJoCo load failed: {exc}")
            self.bus.set_mujoco_status(False, False)
            return False

    def close(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._process is not None and self._process.is_alive():
            self._process.join(timeout=3)
        self._process = None
        self._bridge_thread = None
        self._target_buf = None
        self._state_buf = None
        self._imu_buf = None
        self._stop_event = None
        self.bus.set_mujoco_status(False, False)
        self.bus.set_mujoco_apply_hz(0.0)

    def _bridge_loop(self) -> None:
        """Bridge shared memory arrays <-> SharedStateBus."""
        while self._stop_event is not None and not self._stop_event.is_set():
            if self._process is not None and not self._process.is_alive():
                self.bus.set_mujoco_status(False, False)
                self.bus.set_mujoco_apply_hz(0.0)
                break
            time.sleep(0.02)
            if self._target_buf is not None:
                target = self.bus.get_target()
                np.frombuffer(self._target_buf, dtype=np.float64)[:] = target
            if self._state_buf is not None:
                state = np.frombuffer(self._state_buf, dtype=np.float64)
                self.bus.update_mujoco_state(
                    state[:NUM_JOINTS].copy(),
                    state[NUM_JOINTS:].copy(),
                    time.monotonic(),
                )
            if self._imu_buf is not None:
                imu = np.frombuffer(self._imu_buf, dtype=np.float64)
                self.bus.update_mujoco_imu(imu[:3].copy(), imu[3:].copy())
            self.bus.set_mujoco_apply_hz(500.0)
