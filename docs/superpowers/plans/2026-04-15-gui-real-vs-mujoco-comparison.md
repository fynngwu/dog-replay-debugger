# GUI Real vs MuJoCo Comparison Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enhance the state machine GUI to display real robot and MuJoCo sensor data side-by-side at 50Hz, replacing spinboxes with sliders and adding a batched `get_all` TCP command.

**Architecture:** The data flows from MuJoCo subprocess → shared memory → SharedStateBus → GUI. Robot data flows from C++ StateMachine → TCP → Python client → GUI. A new `get_all` TCP command batches mode+joints+IMU into a single round-trip for 50Hz polling.

**Tech Stack:** Python (PySide6, multiprocessing, numpy), C++ (TCP server, protocol), MuJoCo Python bindings

---

### Task 1: Add `mujoco_imu` field to RuntimeSnapshot

**Files:**
- Modify: `replay_core/types.py:57-79`

- [ ] **Step 1: Add `mujoco_imu` field to RuntimeSnapshot**

In `replay_core/types.py`, add `mujoco_imu: Optional[np.ndarray] = None` between `mujoco_state` and `mujoco_state_age_s`:

```python
@dataclass(slots=True)
class RuntimeSnapshot:
    csv_path: Optional[str]
    sequence_loaded: bool
    total_frames: int
    cursor: int
    playing: bool
    playback_speed: float
    current_target_raw: np.ndarray
    current_target: np.ndarray
    robot_connected: bool
    robot_tx_hz: float
    robot_rx_hz: float
    robot_state: Optional[JointState]
    robot_state_age_s: Optional[float]
    backend_state: Optional[BackendState]
    backend_state_age_s: Optional[float]
    mujoco_loaded: bool
    mujoco_viewer_running: bool
    mujoco_apply_hz: float
    mujoco_state: Optional[JointState]
    mujoco_imu: Optional[np.ndarray] = None
    mujoco_state_age_s: Optional[float]
    log_lines: List[str] = field(default_factory=list)
```

- [ ] **Step 2: Commit**

```bash
git add replay_core/types.py
git commit -m "feat: add mujoco_imu field to RuntimeSnapshot"
```

---

### Task 2: Add `update_mujoco_imu()` to SharedStateBus

**Files:**
- Modify: `replay_core/sync_bus.py`

- [ ] **Step 1: Add `_mujoco_imu` field and `update_mujoco_imu()` method**

In `replay_core/sync_bus.py`:

1. In `__init__` (after `self._mujoco_state` at line 33), add:

```python
self._mujoco_imu: Optional[np.ndarray] = None
```

2. After `update_mujoco_state()` (after line 168), add new method:

```python
def update_mujoco_imu(self, gyro: np.ndarray, gravity: np.ndarray) -> None:
    with self._lock:
        self._mujoco_imu = np.concatenate([
            np.array(gyro, dtype=np.float64),
            np.array(gravity, dtype=np.float64),
        ])
```

3. In `snapshot()` (around line 177, after `mujoco_state_age`), add age calculation:

```python
mujoco_imu_age = None  # no separate timestamp — shares mujoco_state cadence
```

4. In the `RuntimeSnapshot()` constructor call (around line 216, after `mujoco_state=...`), add:

```python
mujoco_imu=None if self._mujoco_imu is None else self._mujoco_imu.copy(),
```

- [ ] **Step 2: Commit**

```bash
git add replay_core/sync_bus.py
git commit -m "feat: add update_mujoco_imu to SharedStateBus"
```

---

### Task 3: Add IMU sensor reading to MuJoCo adapter

**Files:**
- Modify: `adapters/mujoco_adapter.py`

- [ ] **Step 1: Add quaternion helper functions**

Before `_mujoco_subprocess()`, add two functions copied from `sim_record/sim_record.py`:

```python
def _quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    return np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)


def _quat_rotate_inverse_xyzw(quat_xyzw: np.ndarray, vec: np.ndarray) -> np.ndarray:
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
```

- [ ] **Step 2: Add `_imu_buf` parameter to `_mujoco_subprocess()`**

Change the function signature to accept an additional shared memory buffer:

```python
def _mujoco_subprocess(
    xml_path: str,
    target_buf: mp.RawArray,
    state_buf: mp.RawArray,
    imu_buf: mp.RawArray,
    stop_event: mp.Event,
    policy_to_sim: list[int],
    sim_to_policy: list[int],
) -> None:
```

Inside the function, after `state_np = np.frombuffer(state_buf, dtype=np.float64)`, add:

```python
imu_np = np.frombuffer(imu_buf, dtype=np.float64)
```

- [ ] **Step 3: Add IMU sensor reading in the 50Hz write block**

Inside the 50Hz state write block (the `if now - last_state_t >= 0.02:` block), after `state_np[NUM_JOINTS:] = dq[s2p]`, add IMU sensor reading:

```python
# Gyro from angular-velocity sensor
gyro_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
if gyro_sensor_id != -1:
    adr = model.sensor_adr[gyro_sensor_id]
    imu_np[:3] = data.sensordata[adr:adr + 3]

# Projected gravity from orientation quaternion
quat_wxyz = data.qpos[3:7]
quat_xyzw = _quat_wxyz_to_xyzw(quat_wxyz)
imu_np[3:6] = _quat_rotate_inverse_xyzw(quat_xyzw, np.array([0.0, 0.0, -1.0]))
```

- [ ] **Step 4: Add `_imu_buf` to `MujocoAdapter` class**

In `__init__`, add:

```python
self._imu_buf: Optional[mp.RawArray] = None
```

In `load_model()`, after `self._state_buf = mp.RawArray("d", NUM_JOINTS * 2)` (line 109), add:

```python
self._imu_buf = mp.RawArray("d", 6)
```

In the `mp.Process` args tuple, add `self._imu_buf` after `self._state_buf`:

```python
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
```

In `close()`, after `self._state_buf = None` (line 147), add:

```python
self._imu_buf = None
```

- [ ] **Step 5: Bridge IMU data in `_bridge_loop()`**

In `_bridge_loop()`, after the `self.bus.update_mujoco_state(...)` block (after line 169), add:

```python
if self._imu_buf is not None:
    imu = np.frombuffer(self._imu_buf, dtype=np.float64)
    self.bus.update_mujoco_imu(imu[:3].copy(), imu[3:6].copy())
```

- [ ] **Step 6: Commit**

```bash
git add adapters/mujoco_adapter.py
git commit -m "feat: add IMU sensor pipeline to MuJoCo adapter"
```

---

### Task 4: Add `GetAll` command to C++ server

**Files:**
- Modify: `state_machine/protocol.hpp`
- Modify: `state_machine/protocol.cpp`
- Modify: `state_machine/tcp_server.cpp`

- [ ] **Step 1: Add `GetAll` to Command enum and declare `SerializeAll()`**

In `state_machine/protocol.hpp`, change the Command enum:

```cpp
enum class Command { RequestMode, Target, GetMode, GetJoints, GetImu, GetAll, Unknown };
```

Add `SerializeAll()` declaration after `SerializeIMU`:

```cpp
std::string SerializeAll(Mode mode, const float* position, const float* velocity,
                         const float* gyro, const float* gravity, int joint_count);
```

- [ ] **Step 2: Add `get_all` parsing and `SerializeAll()` implementation**

In `state_machine/protocol.cpp`, add `get_all` parsing in `ParseCommand()` (after line 60):

```cpp
else if (op == "get_all")     cmd.type = Command::GetAll;
```

After `SerializeIMU()` (after line 116), add:

```cpp
static std::string ModeToString(Mode mode) {
    switch (mode) {
        case Mode::INIT:    return "INIT";
        case Mode::EXECUTE: return "EXECUTE";
        case Mode::POLICY:  return "POLICY";
        case Mode::STOP:    return "STOP";
    }
    return "UNKNOWN";
}

std::string SerializeAll(Mode mode, const float* position, const float* velocity,
                         const float* gyro, const float* gravity, int joint_count) {
    std::ostringstream oss;
    oss << "{\"mode\":\"" << ModeToString(mode) << "\"";
    oss << ",\"joints\":[";
    for (int i = 0; i < joint_count; ++i) {
        if (i > 0) oss << ",";
        oss << "{\"position\":" << position[i] << ",\"velocity\":" << velocity[i] << "}";
    }
    oss << "],\"imu\":{\"gyro\":[";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) oss << ",";
        oss << gyro[i];
    }
    oss << "],\"gravity\":[";
    for (int i = 0; i < 3; ++i) {
        if (i > 0) oss << ",";
        oss << gravity[i];
    }
    oss << "]}}";
    return oss.str();
}
```

- [ ] **Step 3: Handle `GetAll` in `ProcessCommand()`**

In `state_machine/tcp_server.cpp`, before the `Command::Unknown` case (before line 188), add:

```cpp
case Command::GetAll: {
    auto mode = sm_.GetCurrentMode();
    auto js = sm_.GetJointStates();
    auto imu = sm_.GetIMUData();
    return MakeOkData(SerializeAll(mode, js.position.data(), js.velocity.data(),
                                    imu.angular_velocity.data(),
                                    imu.projected_gravity.data(),
                                    StateMachine::NUM_JOINTS));
}
```

Also update the unknown command error message to include `get_all`:

```cpp
return MakeErrorReply("unknown command, use request_mode/target/get_mode/get_joints/get_imu/get_all");
```

- [ ] **Step 4: Commit**

```bash
git add state_machine/protocol.hpp state_machine/protocol.cpp state_machine/tcp_server.cpp
git commit -m "feat: add get_all batch command to TCP server"
```

---

### Task 5: Add `get_all` to Python client and protocol

**Files:**
- Modify: `state_machine/gui_client/protocol.py`
- Modify: `state_machine/gui_client/client.py`

- [ ] **Step 1: Add `make_get_all()` to protocol**

In `state_machine/gui_client/protocol.py`, after `CMD_GET_IMU` (line 7), add:

```python
CMD_GET_ALL = "get_all"
```

After `make_get_imu()` (after line 30), add:

```python
def make_get_all() -> str:
    return CMD_GET_ALL
```

- [ ] **Step 2: Add `get_all()` to SMClient**

In `state_machine/gui_client/client.py`, after `get_imu()` (after line 71), add:

```python
def get_all(self) -> dict:
    return self._send_recv(protocol.make_get_all())
```

- [ ] **Step 3: Commit**

```bash
git add state_machine/gui_client/protocol.py state_machine/gui_client/client.py
git commit -m "feat: add get_all command to Python protocol and client"
```

---

### Task 6: Replace spinboxes with sliders in ControlPanel

**Files:**
- Modify: `state_machine/gui_client/control_panel.py`

- [ ] **Step 1: Replace QDoubleSpinBox with QSlider**

Replace the import line to add `QSlider`:

```python
from PySide6.QtWidgets import (
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
```

Remove `QDoubleSpinBox` from imports (it is still used by `speed_spin`, so keep it):

```python
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
```

Replace the target spinbox creation block (lines 84-98) with:

```python
self.target_sliders: list[QSlider] = []
self.target_val_labels: list[QLabel] = []
for i in range(12):
    row = QHBoxLayout()
    row.setSpacing(6)
    name_label = QLabel(JOINT_NAMES[i])
    name_label.setFixedWidth(72)
    slider = QSlider(Qt.Orientation.Horizontal)
    slider.setRange(-31416, 31416)
    slider.setValue(0)
    self.target_sliders.append(slider)
    val_label = QLabel("0.0000")
    val_label.setFixedWidth(64)
    val_label.setObjectName("val")
    self.target_val_labels.append(val_label)
    row.addWidget(name_label)
    row.addWidget(slider)
    row.addWidget(val_label)
    target_layout.addLayout(row)
```

Add a helper property to get all slider values as floats (replace old `target_spins` usage). This goes after the `target_box` section, or as a method:

```python
def get_target_values(self) -> list[float]:
    return [s.value() / 10000.0 for s in self.target_sliders]
```

Note: `-31416 / 10000.0 = -3.1416 ≈ -pi`, `31416 / 10000.0 = 3.1416 ≈ pi`. Resolution = 0.0001 rad.

- [ ] **Step 2: Connect slider `valueChanged` to update val labels**

After the slider creation loop, connect signals:

```python
for i in range(12):
    self.target_sliders[i].valueChanged.connect(
        lambda val, idx=i: self.target_val_labels[idx].setText(f"{val / 10000.0:.4f}")
    )
```

- [ ] **Step 3: Commit**

```bash
git add state_machine/gui_client/control_panel.py
git commit -m "feat: replace joint target spinboxes with sliders"
```

---

### Task 7: Extend status panel with Target/MuJoCo columns and dual IMU

**Files:**
- Modify: `state_machine/gui_client/status_panel.py`

- [ ] **Step 1: Expand joint table to 7 columns**

Replace the table creation (lines 50-66) with:

```python
self.joint_table = QTableWidget(12, 7)
self.joint_table.setHorizontalHeaderLabels(
    ["Joint", "Target", "Robot Pos", "Mujoco Pos", "Robot Vel", "Mujoco Vel", ""]
)
self.joint_table.horizontalHeader().setStretchLastSection(True)
self.joint_table.setColumnWidth(0, 72)
self.joint_table.setColumnWidth(1, 72)
self.joint_table.setColumnWidth(2, 80)
self.joint_table.setColumnWidth(3, 80)
self.joint_table.setColumnWidth(4, 80)
self.joint_table.setColumnWidth(5, 80)
self.joint_table.verticalHeader().setVisible(False)
self.joint_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
self.joint_table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)

for i, name in enumerate(JOINT_NAMES):
    self.joint_table.setItem(i, 0, QTableWidgetItem(name))
    self.joint_table.setItem(i, 1, QTableWidgetItem("0.0000"))
    self.joint_table.setItem(i, 2, QTableWidgetItem("0.0000"))
    self.joint_table.setItem(i, 3, QTableWidgetItem("0.0000"))
    self.joint_table.setItem(i, 4, QTableWidgetItem("0.0000"))
    self.joint_table.setItem(i, 5, QTableWidgetItem("0.0000"))
```

- [ ] **Step 2: Replace IMU panel with side-by-side Robot and MuJoCo groups**

Replace the IMU box creation (lines 70-93) with:

```python
imu_splitter = QSplitter(Qt.Orientation.Horizontal)

robot_imu_box = QGroupBox("Robot IMU")
robot_imu_layout = QGridLayout(robot_imu_box)
robot_imu_layout.setContentsMargins(12, 12, 12, 12)
robot_imu_layout.setSpacing(6)

mujoco_imu_box = QGroupBox("MuJoCo IMU")
mujoco_imu_layout = QGridLayout(mujoco_imu_box)
mujoco_imu_layout.setContentsMargins(12, 12, 12, 12)
mujoco_imu_layout.setSpacing(6)

self.robot_imu_labels: dict[str, QLabel] = {}
self.mujoco_imu_labels: dict[str, QLabel] = {}
imu_fields = [
    ("Gyro X", "gyro_x"),
    ("Gyro Y", "gyro_y"),
    ("Gyro Z", "gyro_z"),
    ("Gravity X", "grav_x"),
    ("Gravity Y", "grav_y"),
    ("Gravity Z", "grav_z"),
]
for row, (label, key) in enumerate(imu_fields):
    name_lbl = QLabel(label)
    name_lbl.setObjectName("info_label")
    val_lbl = QLabel("0.0000")
    val_lbl.setObjectName("val")
    self.robot_imu_labels[key] = val_lbl
    robot_imu_layout.addWidget(name_lbl, row, 0)
    robot_imu_layout.addWidget(val_lbl, row, 1)

    name_lbl2 = QLabel(label)
    name_lbl2.setObjectName("info_label")
    val_lbl2 = QLabel("0.0000")
    val_lbl2.setObjectName("val")
    self.mujoco_imu_labels[key] = val_lbl2
    mujoco_imu_layout.addWidget(name_lbl2, row, 0)
    mujoco_imu_layout.addWidget(val_lbl2, row, 1)

imu_splitter.addWidget(robot_imu_box)
imu_splitter.addWidget(mujoco_imu_box)
```

Also add `QSplitter` to imports if not already present:

```python
from PySide6.QtWidgets import (
    QGroupBox,
    QGridLayout,
    QLabel,
    QPlainTextEdit,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
```

Replace the old `splitter.addWidget(imu_box)` (line 93) with:

```python
splitter.addWidget(imu_splitter)
```

- [ ] **Step 3: Update `update_joints()` method signature and body**

Replace the `update_joints()` method (lines 111-115) with:

```python
def update_joints(self, target: list[float] | None = None,
                  robot_pos: list[float] | None = None,
                  robot_vel: list[float] | None = None,
                  mujoco_pos: list[float] | None = None,
                  mujoco_vel: list[float] | None = None) -> None:
    with QSignalBlocker(self.joint_table):
        if target is not None:
            for i in range(12):
                self.joint_table.item(i, 1).setText(f"{target[i]:.4f}")
        if robot_pos is not None:
            for i in range(12):
                self.joint_table.item(i, 2).setText(f"{robot_pos[i]:.4f}")
        if mujoco_pos is not None:
            for i in range(12):
                self.joint_table.item(i, 3).setText(f"{mujoco_pos[i]:.4f}")
        if robot_vel is not None:
            for i in range(12):
                self.joint_table.item(i, 4).setText(f"{robot_vel[i]:.4f}")
        if mujoco_vel is not None:
            for i in range(12):
                self.joint_table.item(i, 5).setText(f"{mujoco_vel[i]:.4f}")
```

- [ ] **Step 4: Update `update_imu()` to handle dual IMU**

Replace the `update_imu()` method (lines 117-123) with:

```python
def update_imu(self, robot_gyro: list[float] | None = None,
               robot_gravity: list[float] | None = None,
               mujoco_gyro: list[float] | None = None,
               mujoco_gravity: list[float] | None = None) -> None:
    if robot_gyro is not None:
        self.robot_imu_labels["gyro_x"].setText(f"{robot_gyro[0]:.4f}")
        self.robot_imu_labels["gyro_y"].setText(f"{robot_gyro[1]:.4f}")
        self.robot_imu_labels["gyro_z"].setText(f"{robot_gyro[2]:.4f}")
    if robot_gravity is not None:
        self.robot_imu_labels["grav_x"].setText(f"{robot_gravity[0]:.4f}")
        self.robot_imu_labels["grav_y"].setText(f"{robot_gravity[1]:.4f}")
        self.robot_imu_labels["grav_z"].setText(f"{robot_gravity[2]:.4f}")
    if mujoco_gyro is not None:
        self.mujoco_imu_labels["gyro_x"].setText(f"{mujoco_gyro[0]:.4f}")
        self.mujoco_imu_labels["gyro_y"].setText(f"{mujoco_gyro[1]:.4f}")
        self.mujoco_imu_labels["gyro_z"].setText(f"{mujoco_gyro[2]:.4f}")
    if mujoco_gravity is not None:
        self.mujoco_imu_labels["grav_x"].setText(f"{mujoco_gravity[0]:.4f}")
        self.mujoco_imu_labels["grav_y"].setText(f"{mujoco_gravity[1]:.4f}")
        self.mujoco_imu_labels["grav_z"].setText(f"{mujoco_gravity[2]:.4f}")
```

- [ ] **Step 5: Commit**

```bash
git add state_machine/gui_client/status_panel.py
git commit -m "feat: extend joint table with Target/MuJoCo columns and dual IMU panel"
```

---

### Task 8: Update main.py for 50Hz batch polling and MuJoCo display

**Files:**
- Modify: `state_machine/gui_client/main.py`

- [ ] **Step 1: Change timer to 50Hz**

In `__init__`, change line 68:

```python
self.refresh_timer.start(20)
```

- [ ] **Step 2: Update `_send_target()` to use sliders**

Replace `_send_target()` (lines 142-149):

```python
def _send_target(self) -> None:
    joints = self.controls.get_target_values()
    self._log("sending target...")
    reply = self.client.send_target(joints)
    if reply.get("ok"):
        self._log(reply.get("msg", ""))
    else:
        self._log(f"error: {reply.get('msg', '')}")
```

- [ ] **Step 3: Add `_last_target` tracking in `__init__`**

After `self._current_frame: int = 0` (line 60), add:

```python
self._last_target: list[float] = [0.0] * 12
```

- [ ] **Step 4: Update `_send_target()` to store last target**

After `reply.get("ok")` succeeds, store the target:

```python
def _send_target(self) -> None:
    joints = self.controls.get_target_values()
    self._log("sending target...")
    reply = self.client.send_target(joints)
    if reply.get("ok"):
        self._last_target = joints
        self._log(reply.get("msg", ""))
    else:
        self._log(f"error: {reply.get('msg', '')}")
```

- [ ] **Step 5: Replace `_refresh()` with batched `get_all` polling**

Replace the `_refresh()` method (lines 248-291) with:

```python
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
        return  # skip TCP polling during replay — worker thread owns the socket

    # MuJoCo data from SharedStateBus (always available, no TCP needed)
    mujoco_pos = None
    mujoco_vel = None
    mujoco_gyro = None
    mujoco_gravity = None
    if snapshot.mujoco_state is not None:
        mujoco_pos = snapshot.mujoco_state.positions.tolist()
        mujoco_vel = snapshot.mujoco_state.velocities.tolist()
    if snapshot.mujoco_imu is not None:
        mujoco_gyro = snapshot.mujoco_imu[:3].tolist()
        mujoco_gravity = snapshot.mujoco_imu[3:6].tolist()

    self.status.update_joints(
        target=self._last_target,
        mujoco_pos=mujoco_pos,
        mujoco_vel=mujoco_vel,
    )

    self.status.update_imu(
        mujoco_gyro=mujoco_gyro,
        mujoco_gravity=mujoco_gravity,
    )

    if not self.client.is_connected:
        return

    # Single batched TCP call for mode + joints + IMU
    all_reply = self.client.get_all()
    if not all_reply.get("ok"):
        return

    data = all_reply.get("data", {})

    # Mode
    mode_str = data.get("mode", "").replace("current mode: ", "")
    if not mode_str and all_reply.get("msg"):
        mode_str = all_reply.get("msg", "").replace("current mode: ", "")
    self.status.update_mode(mode_str)

    # Joints
    joints = data.get("joints", [])
    if len(joints) == 12:
        robot_pos = [j.get("position", 0.0) for j in joints]
        robot_vel = [j.get("velocity", 0.0) for j in joints]
        self.status.update_joints(
            robot_pos=robot_pos,
            robot_vel=robot_vel,
        )

    # IMU
    imu = data.get("imu", {})
    robot_gyro = imu.get("gyro")
    robot_gravity = imu.get("gravity")
    if robot_gyro and robot_gravity:
        self.status.update_imu(
            robot_gyro=robot_gyro,
            robot_gravity=robot_gravity,
        )
```

- [ ] **Step 6: Commit**

```bash
git add state_machine/gui_client/main.py
git commit -m "feat: 50Hz batch polling via get_all, MuJoCo display in GUI"
```

---

### Task 9: Build and smoke test

- [ ] **Step 1: Build the C++ server**

```bash
cd /home/wufy/github_respository/dog_project/dog_replay_debugger/state_machine
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```

Expected: compilation succeeds with no errors.

- [ ] **Step 2: Verify Python imports work**

```bash
cd /home/wufy/github_respository/dog_project/dog_replay_debugger/state_machine/gui_client
python3 -c "from control_panel import ControlPanel; from status_panel import StatusPanel; print('imports ok')"
```

Expected: prints `imports ok` (may fail if no display, that's fine — test on robot).

- [ ] **Step 3: Commit final state**

If any adjustments were needed during build:

```bash
git add -A
git commit -m "fix: build adjustments for get_all command"
```

---

## Self-Review

1. **Spec coverage:**
   - Slider-based joint control → Task 6
   - Joint table with Target/MuJoCo columns → Task 7
   - Dual IMU panel → Task 7
   - MuJoCo adapter IMU pipeline → Task 3
   - SharedStateBus mujoco_imu → Task 2
   - RuntimeSnapshot mujoco_imu → Task 1
   - Server-side get_all C++ command → Task 4
   - Client-side get_all Python → Task 5
   - 50Hz polling in main.py → Task 8
   - All 11 files from spec covered.

2. **Placeholder scan:** No TBD, TODO, or vague steps found.

3. **Type consistency:** `update_joints(target, robot_pos, robot_vel, mujoco_pos, mujoco_vel)` matches across status_panel.py (Task 7) and main.py (Task 8). `update_imu(robot_gyro, robot_gravity, mujoco_gyro, mujoco_gravity)` matches across the same. `get_all()` return format matches SerializeAll() JSON structure. `_imu_buf` parameter flows through mujoco_adapter.py consistently.

4. **Dependency order:** Tasks 1-2 (Python data types, no dependencies) → Task 3 (depends on Task 2 bus API) → Task 4 (C++, independent) → Task 5 (Python client, depends on Task 4 conceptually but code independent) → Task 6 (GUI, depends on Task 5 interface) → Task 7 (GUI, depends on Tasks 2,3) → Task 8 (main.py, depends on all above) → Task 9 (build/test).
