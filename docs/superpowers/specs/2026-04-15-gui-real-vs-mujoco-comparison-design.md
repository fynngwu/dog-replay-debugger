# Design: GUI Real Robot vs MuJoCo Real-Time Comparison

## Problem

The real robot cannot run the locomotion policy properly. The existing GUI only displays real robot sensor data (joint states, IMU) and uses 10Hz polling via 3 sequential TCP round-trips. There is no way to compare real robot sensors against MuJoCo simulation in real time.

## Goal

Enhance the state machine GUI so that real robot sensor data and MuJoCo simulation data are displayed side-by-side in real time at 50Hz, enabling direct comparison while sending joint targets via sliders.

## Design

### 1. Slider-based joint target control

Replace the 12 `QDoubleSpinBox` inputs in `control_panel.py` with `QSlider` widgets.

- Range: -31416 to 31416 integers, mapped to -pi to pi radians (~0.0002 rad resolution)
- Layout per row: `QLabel(name, 72px) | QSlider(flex) | QLabel(value, 64px, read-only)`
- Slider `valueChanged` signal updates the read-only label and emits `target_changed` with the float value

### 2. Joint states table with Target and MuJoCo columns

Expand the `QTableWidget` in `status_panel.py` from 4 columns to 7:

| Joint | Target (rad) | Robot Pos (rad) | Mujoco Pos (rad) | Robot Vel (rad/s) | Mujoco Vel (rad/s) | |

- New method: `update_joints(target, robot_pos, robot_vel, mujoco_pos, mujoco_vel)`
- **Target**: last value sent via TCP (stored client-side in `main.py`)
- **Robot Pos/Vel**: from `get_all` TCP response (real robot data)
- **Mujoco Pos/Vel**: from `SharedStateBus.mujoco_state` (positions are relative to default, in policy order — same as policy runner sees)

### 3. Side-by-side IMU panel

Redesign the IMU section in `status_panel.py` as two groups side by side:

- **Robot IMU**: Gyro X/Y/Z, Gravity X/Y/Z (from `get_all` TCP response)
- **MuJoCo IMU**: Gyro X/Y/Z, Gravity X/Y/Z (from `SharedStateBus.mujoco_imu`)
- New method: `update_imu(robot_gyro, robot_gravity, mujoco_gyro, mujoco_gravity)`

### 4. MuJoCo adapter IMU data pipeline

Add IMU sensor data to the MuJoCo adapter (`adapters/mujoco_adapter.py`):

- **Shared memory**: new `_imu_buf = mp.RawArray("d", 6)` — gyro(3) + projected_gravity(3)
- **Subprocess** (at 50Hz, alongside joint state write):
  - Gyro: `mj_name2id(model, mjOBJ_SENSOR, "angular-velocity")` then `data.sensordata[adr:adr+3]` — identical to `sim_record.py:297`
  - Projected gravity: read `data.qpos[3:7]` (MuJoCo wxyz), convert to xyzw via `quat_wxyz_to_xyzw()`, then `quat_rotate_inverse_xyzw(quat_xyzw, [0,0,-1])` — identical to `sim_record.py:247-250`
  - Copy `quat_wxyz_to_xyzw` and `quat_rotate_inverse_xyzw` functions from `sim_record.py`
- **Bridge loop**: read `_imu_buf`, call `bus.update_mujoco_imu(gyro, gravity)`

### 5. SharedStateBus MuJoCo IMU field

Add to `replay_core/sync_bus.py`:
- `_mujoco_imu: Optional[np.ndarray]` of shape (6,) — [gyro(3), projected_gravity(3)]
- `update_mujoco_imu(gyro, gravity)` method
- Include `mujoco_imu` in `snapshot()` return

Add to `replay_core/types.py`:
- `mujoco_imu: Optional[np.ndarray]` field in `RuntimeSnapshot`

### 6. 50Hz batch polling via `get_all` command

Change polling from 10Hz with 3 sequential TCP calls to 50Hz with 1 batched call.

**Client-side** (`gui_client/main.py`):
- Timer interval: 100ms -> 20ms
- Single `client.get_all()` call replaces `get_mode()` + `get_joints()` + `get_imu()`
- Read MuJoCo state from `bus.snapshot().mujoco_state` and `bus.snapshot().mujoco_imu`
- Store `_last_target` client-side for the Target column

**Client protocol** (`gui_client/client.py`, `gui_client/protocol.py`):
- New `make_get_all()` command builder
- New `client.get_all()` method returning `(mode, positions, velocities, gyro, gravity)`

**Server-side** (`state_machine/protocol.hpp`, `protocol.cpp`, `tcp_server.cpp`):
- Add `GetAll` to `Command` enum
- `SerializeAll()` returns JSON: `{"mode": "...", "joints": [...], "imu": {...}}`
  - `joints`: array of 12 `{"position": float, "velocity": float}` (same format as existing `get_joints`)
  - `imu`: `{"gyro_x": float, "gyro_y": float, "gyro_z": float, "grav_x": float, "grav_y": float, "grav_z": float}` (same format as existing `get_imu`)
- Single DogDriver read, single TCP reply

## Files Modified

1. `state_machine/gui_client/control_panel.py` — sliders replace spinboxes
2. `state_machine/gui_client/status_panel.py` — extended table + dual IMU
3. `state_machine/gui_client/main.py` — 50Hz timer, batch polling, MuJoCo display
4. `state_machine/gui_client/client.py` — `get_all()` method
5. `state_machine/gui_client/protocol.py` — `make_get_all()` builder
6. `replay_core/sync_bus.py` — `update_mujoco_imu()`
7. `replay_core/types.py` — `mujoco_imu` in RuntimeSnapshot
8. `adapters/mujoco_adapter.py` — IMU sensor reading + shared memory
9. `state_machine/protocol.hpp` — `GetAll` command enum
10. `state_machine/protocol.cpp` — `SerializeAll()`, parse `get_all`
11. `state_machine/tcp_server.cpp` — handle `get_all`

## Implementation Order

1. SharedStateBus + types (foundation, no GUI/C++ dependencies)
2. MuJoCo adapter IMU pipeline (depends on step 1)
3. Server-side `get_all` C++ command (independent of Python changes)
4. Client + protocol Python (`get_all()` method, `make_get_all()`)
5. GUI: sliders, table columns, IMU panel (depends on all above)

## Verification

1. Connect GUI to real robot (or server with mock)
2. Load MuJoCo model in GUI
3. Send target joints via sliders — verify MuJoCo tracks and real robot responds
4. Check joint table: Target, Robot Pos, MuJoCo Pos columns show correct values
5. Check IMU panel: Robot and MuJoCo gyro/gravity update side by side
6. Verify polling is smooth at 50Hz (no GUI lag)
7. During replay mode: MuJoCo IMU continues updating, robot IMU shows real data
