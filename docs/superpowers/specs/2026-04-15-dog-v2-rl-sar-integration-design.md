# dog_v2 — rl_sar Integration Design

## Goal

Integrate the custom 12-DOF quadruped (Robstride motors + CAN bus + WIT IMU) into the rl_sar framework for on-robot RL policy deployment, without ROS. Reuse the existing `DogDriver` library by extending it with a full MIT command interface.

## Target Platform

- NVIDIA Jetson (ARM64)
- Inference backend: ONNX Runtime (.onnx)
- No ROS dependency

## Approach

**Plan B: Extend DogDriver** — add a `SetAllJointCommands(pos, vel, tau, kp, kd)` method that exposes the full MIT command format (the underlying `RobstrideController::SendMITCommand` already supports all parameters). The rl_sar adapter calls this new method in `SetCommand()`.

---

## New Files

### In rl_sar project

| File | Purpose |
|------|---------|
| `src/rl_sar/fsm_robot/fsm_dog_v2.hpp` | FSM states (Passive, GetUp, GetDown, Locomotion), factory, registration |
| `src/rl_sar/include/rl_real_dog_v2.hpp` | `RL_Real` class declaration (inherits `RL`) |
| `src/rl_sar/src/rl_real_dog_v2.cpp` | Implementation: `GetState`, `SetCommand`, `Forward`, `RobotControl`, `RunModel`, constructor |
| `src/rl_sar/fsm_robot/fsm_all.hpp` | Add `#include "fsm_dog_v2.hpp"` |
| `policy/dog_v2/base.yaml` | Robot base config (DOF, default pose, control params) |
| `policy/dog_v2/legged_gym/config.yaml` | Policy config (observations, action scale, model path) |

### Modified files

| File | Change |
|------|--------|
| `src/rl_sar/CMakeLists.txt` | Add `rl_real_dog_v2` executable target, link `dog_driver` |
| `driver/include/dog_driver.hpp` | Add `quaternion` field to `IMUData`, add `SetAllJointCommands()` |
| `driver/src/dog_driver.cpp` | Implement `SetAllJointCommands()`, populate quaternion in `GetIMUData()` |

### No changes to rl_sar core

The `rl_sdk`, `fsm`, `observation_buffer`, `inference_runtime`, and `loop` libraries remain untouched.

---

## DogDriver Extensions

### 1. IMUData gains quaternion field

```cpp
struct IMUData {
    std::array<float, 3> angular_velocity;
    std::array<float, 3> projected_gravity;
    std::array<float, 4> quaternion;  // NEW: [w, x, y, z]
};
```

Populated from the existing `IMUComponent` quaternion buffer (already read by the WIT SDK callback, just not exposed).

### 2. New method: SetAllJointCommands

```cpp
int SetAllJointCommands(
    const std::array<float, 12>& pos,   // joint-space target position
    const std::array<float, 12>& vel,   // feedforward velocity
    const std::array<float, 12>& tau,   // feedforward torque
    const std::array<float, 12>& kp,    // position gain
    const std::array<float, 12>& kd     // velocity gain
);
```

Implementation: for each joint i, perform joint-to-motor coordinate conversion (knee * gear_ratio, direction, offset), then call `motor_controller_->SendMITCommand(i, motor_pos, motor_vel, kp[i], kd[i], tau[i])`.

Coordinate conversion is factored into a shared private helper so both `SetAllJointPositions` and `SetAllJointCommands` reuse the same logic.

---

## rl_real_dog_v2 — Hardware Adapter

### Class Hierarchy

```
RL (rl_sdk base class)
  └── RL_Real (rl_real_dog_v2.cpp)
        ├── holds DogDriver& driver_
        ├── overrides Forward(), GetState(), SetCommand()
        └── implements RobotControl(), RunModel()
```

### Constructor

1. Set `robot_name = "dog_v2"`, `ang_vel_axis = "body"`
2. Call `ReadYaml("dog_v2", "base.yaml")`
3. Construct `DogDriver` (opens CAN buses, enables motors, starts background threads)
4. Call `FSMManager::GetInstance().CreateFSM("dog_v2", this)` to create FSM
5. Call `InitJointNum(12)`, `InitOutputs()`, `InitControl()`
6. Start 3 LoopFunc threads (no UDP threads — CAN handled by DogDriver internally)

### GetState — Read hardware

- `DogDriver::GetJointStates()` → `state->motor_state.q[]`, `dq[]`, `tauEst[]=0`
- `DogDriver::GetIMUData()` → `state->imu.quaternion[]`, `state->imu.gyroscope[]`
- Keyboard/gamepad input is managed by rl_sdk's `control` struct (keyboard loop writes to it)

### SetCommand — Send motor commands

- Map `RobotCommand.motor_command` arrays → `DogDriver::SetAllJointCommands(pos, vel, tau, kp, kd)`
- This sends the full MIT command (position + velocity feedforward + torque feedforward + PD gains) to all 12 motors via CAN

### Forward — Policy inference

Identical to A1 implementation:
- `std::try_to_lock` on `model_mutex` (non-blocking)
- `ComputeObservation()` → build obs vector from YAML config
- `model->forward(obs)` → action vector
- Clamp to `clip_actions_lower/upper`
- Return previous actions if mutex unavailable

### RobotControl (control loop callback)

```
GetState(&robot_state)
  → StateController(&robot_state, &robot_command)
      → FSM.Run()
  → control.ClearInput()
→ SetCommand(&robot_command)
```

### RunModel (inference loop callback)

```
obs.ang_vel    = robot_state.imu.gyroscope
obs.commands   = {control.x, control.y, control.yaw}
obs.base_quat  = robot_state.imu.quaternion
obs.dof_pos    = robot_state.motor_state.q
obs.dof_vel    = robot_state.motor_state.dq
actions        = Forward()
ComputeOutput(actions, output_dof_pos, output_dof_vel, output_dof_tau)
push output_dof_pos/vel/tau into tbb::concurrent_queue
```

### Thread Model

| Thread | Period | Purpose |
|--------|--------|---------|
| `loop_control` | 5ms (200Hz) | RobotControl: read state → FSM → send commands |
| `loop_rl` | 20ms (50Hz) | RunModel: observation → inference → push to queue |
| `loop_keyboard` | 50ms (20Hz) | Read keyboard input for velocity commands and FSM transitions |

No separate send/receive threads needed — DogDriver handles CAN communication internally with its own background threads.

---

## FSM States

Four states matching the rl_sar pattern:

### Passive (initial state)
- `Run()`: set `kp=0, kd=8, tau=0` on all joints (damping-only mode)
- `CheckChange()`: `0` or `A` → GetUp

### GetUp
- `Run()`: linear interpolation from current position to `default_dof_pos` over 2.5s using `Interpolate()` helper
- Uses `fixed_kp`/`fixed_kd` from base.yaml during interpolation
- `CheckChange()`: interpolation complete + `1` → Locomotion

### Locomotion
- `Enter()`: call `rl.InitRL("dog_v2/legged_gym")` — reads policy config, loads ONNX model, initializes observation history
- `Run()`: call `RLControl()` — pops position/velocity from tbb queue, writes `rl_kp`/`rl_kd` gains
- `Exit()`: set `rl_init_done = false`
- `CheckChange()`: `P` → Passive, `9` → GetDown

### GetDown
- `Run()`: linear interpolation from `default_dof_pos` to zero over 2s
- `CheckChange()`: interpolation complete → Passive

---

## YAML Configuration

### policy/dog_v2/base.yaml

```yaml
num_of_dofs: 12
default_dof_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
dt: 0.005
decimation: 4
fixed_kp: 20.0
fixed_kd: 0.5
```

### policy/dog_v2/legged_gym/config.yaml

```yaml
model_name: "policy.onnx"
observations: ["ang_vel", "gravity_vec", "commands", "dof_pos", "dof_vel", "actions"]
clip_obs: 100.0
clip_actions_lower: [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]
clip_actions_upper: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
action_scale: 0.25
rl_kp: 20.0
rl_kd: 0.5
lin_vel_scale: 2.0
ang_vel_scale: 0.25
commands_scale: [2.0, 2.0, 2.0]
dof_pos_scale: 1.0
dof_vel_scale: 0.05
torque_limits: [17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0, 17.0]
observations_history: []
```

Note: `default_dof_pos` and `torque_limits` are placeholders (all zeros / 17.0). Adjust to match actual robot hardware.

---

## CMake Integration

Add to `src/rl_sar/CMakeLists.txt`:

```cmake
# DogDriver dependency
set(DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../driver")
find_library(DOG_DRIVER dog_driver PATHS "${DRIVER_DIR}/build")

if(DOG_DRIVER)
    add_executable(rl_real_dog_v2 src/rl_real_dog_v2.cpp)
    target_include_directories(rl_real_dog_v2 PRIVATE "${DRIVER_DIR}/include")
    target_link_libraries(rl_real_dog_v2
        ${DOG_DRIVER}
        rl_sdk
        observation_buffer
        ${YAML_CPP_LIBRARIES}
        pthread
    )
endif()
```

---

## Build & Run

```bash
# 1. Build DogDriver
cd driver && mkdir -p build && cd build && cmake .. && make -j$(nproc)

# 2. Download ONNX Runtime + build rl_sar
cd rl_sar && bash build.sh -m

# 3. Reconfigure with DogDriver path
cd rl_sar && cmake src/rl_sar/ -B cmake_build -DUSE_CMAKE=ON \
    -DCMAKE_PREFIX_PATH="${PWD}/../../driver/build"
cmake --build cmake_build -j$(nproc)

# 4. Run (needs root for CAN bus)
sudo ./cmake_build/bin/rl_real_dog_v2

# Keyboard controls:
# 0 → Passive → GetUp
# 1 → GetUp → Locomotion
# P → Any → Passive
# WASD → velocity commands
```

---

## Data Flow

```
Keyboard (WASD/0/1/P)
    │
    ▼
loop_keyboard (50ms) ──→ control struct (x, y, yaw)
                               │
                               ▼
loop_control (5ms, 200Hz):
  DogDriver.GetJointStates() ──→ RobotState.motor_state
  DogDriver.GetIMUData()    ──→ RobotState.imu
                                      │
                                      ▼
                              StateController()
                                FSM.Run()
                                ┌─ Passive: kp=0, kd=8, tau=0
                                ├─ GetUp: Interpolate → fixed_kp/kd
                                └─ Locomotion: RLControl()
                                      │  (pop from tbb queue)
                                      │  ┌──────────────────────┐
                                      │  │  loop_rl (20ms, 50Hz) │
                                      │  │  obs ← RobotState     │
                                      │  │  actions ← Forward()  │
                                      │  │  output ← ComputeOutput│
                                      │  │  push to queue        │
                                      │  └──────────────────────┘
                                      ▼
                              RobotCommand
                                      │
                                      ▼
                              DogDriver.SetAllJointCommands()
                                → joint→motor conversion
                                → CAN bus → Robstride motors
```

## Key Design Decisions

1. **No ROS**: standalone CMake build with `USE_CMAKE=ON`
2. **ONNX Runtime on Jetson ARM64**: official support, no TensorRT custom backend needed
3. **Reuse DogDriver**: only extend with `SetAllJointCommands()` — no rewrite
4. **Control at 200Hz, inference at 50Hz**: decoupled via tbb concurrent queues
5. **Non-blocking inference**: `std::try_to_lock` prevents control thread stalls
6. **Keyboard input only**: standard rl_sar keyboard interface (WASD + number keys)
7. **Damping-only passive mode**: `kp=0, kd=8` — motor stays enabled but compliant
8. **YAML-driven observations**: easily switch policies without recompilation
9. **No observation history initially**: can be added later via `observations_history` config
