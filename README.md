# Dog Replay Debugger

A minimal **CLI-first** replay tool for comparing one recorded policy trajectory against:

1. a real robot connected through the existing twin-agent command/state protocol, and
2. a local MuJoCo mirror driven by the **same local replay cursor**.

This project intentionally removes the old multi-owner replay design. The only replay truth source is the local Python `ReplayEngine`.

## What this tool does

- loads a replay CSV **locally** on the client machine
- maintains one local replay cursor
- supports `step`, `prev`, `seek(frame)`, `start(frame)` and `stop`
- sends the active frame target to the robot through `set_joint <12 floats>`
- applies the same target to a local MuJoCo mirror
- shows at ~50 Hz:
  - target joints from CSV
  - MuJoCo joint positions
  - robot joint positions
  - target-vs-sim and target-vs-robot errors

## Removed on purpose

This project does **not** upload CSV to Jetson and does **not** use any remote replay state:

- no remote `load_replay_csv`
- no remote `replay_start/stop/seek/step/prev`
- no remote replay cursor
- no double playback loops

## Replay CSV columns

Supported target columns, in priority order:

1. `target_rel_0 .. target_rel_11`
2. `scaled_action_0 .. scaled_action_11`

Supported time columns:

1. `timestamp_ms`
2. `sim_time`
3. fallback to row index × estimated frame dt

## Run GUI

```bash
uv run python run_gui.py
```

## Run CLI

```bash
uv run python run_cli.py
```

Inside the CLI, type `help`.

## Recommended usage order

1. Start GUI or CLI.
2. Load CSV.
3. Load MuJoCo XML and start the viewer.
4. Connect robot.
5. Run `init` once on the robot if needed.
6. Use `step`, `prev`, `seek N`, or `start N`.
7. Watch target / MuJoCo / robot differences in the table and plot.

## Safety notes

- This tool sends joint targets directly with `set_joint`.
- Make sure the robot side is already in the expected enabled / initialized state before stepping through frames.
- Start with a safe trajectory and keep emergency stop access available.

## File layout

```text
run_gui.py                      # Qt entry point
run_cli.py                      # interactive CLI entry point
replay_core/                    # replay state, CSV loading, snapshots
adapters/                       # robot and MuJoCo adapters
cli/                            # CLI session helpers
gui/                            # Qt widgets and plotting
examples/sample_replay.csv      # tiny local test trajectory
driver/                         # DogDriver shared library (CAN motor + WIT IMU)
state_machine/                  # StateMachine + TCP server + Python GUI client
state_machine/test/sm_test.cpp  # Hardware test CLI (links DogDriver directly)
```

## State Machine Module

C++ state machine that manages robot modes (INIT / EXECUTE / POLICY / STOP) and exposes a TCP server for remote control.

### Hardware test tool (`sm_test`)

CLI tool for direct hardware testing without TCP. Links `DogDriver` directly.

```bash
# Build (from project root)
mkdir -p build && cd build && cmake ../state_machine && make sm_test -j$(nproc)

# Or standalone
./state_machine/test/build.sh
```

```text
Usage: sm_test <command> [args...]

Commands:
  init                Enable motors + auto-report + interpolate to zero (2.5s)
  autoreport          Enable auto-report on all joints
  joints              Read all joint positions/velocities once
  joints --stream     Stream joints at ~10 Hz (Ctrl+C to stop)
  imu                 Read IMU data once
  imu --stream        Stream IMU data at ~10 Hz (Ctrl+C to stop)
  set_joint <idx> <rad>  Set single joint position
  enable              Enable all motors
  disable             Disable all motors
  online              Check which motors are online
  info                Print driver info (IMU status, motor status)
```

### Test checklist

| Test | Command | Status |
|------|---------|--------|
| Motor CAN connectivity | `sm_test online` | Passed (12/12) |
| IMU serial connectivity | `sm_test info` | Passed (CH340 @ 115200 baud) |
| Joint position read | `sm_test joints` | Passed |
| Joint velocity read | `sm_test joints` | Passed |
| IMU gyro read | `sm_test imu` | Passed |
| IMU gravity read | `sm_test imu` | Passed |
| Joint real-time stream | `sm_test joints --stream` | Passed |
| IMU real-time stream | `sm_test imu --stream` | Passed |
| Motor enable/disable | `sm_test enable` / `sm_test disable` | Passed |
| Init sequence (home) | `sm_test init` | Passed |
| Single joint control | `sm_test set_joint 0 0.3` | Passed |
| Auto-report enable | `sm_test autoreport` | Passed |
