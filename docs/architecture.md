# Architecture

## Single replay truth source

The local `ReplayEngine` is the **only** owner of replay cursor state.

- CSV is loaded locally.
- `step`, `prev`, `seek`, and `start` all modify the local cursor.
- The robot never stores replay state.
- MuJoCo never stores replay state.

## Data flow

```text
CSV -> ReplayEngine(cursor,target)
          |                |
          |                +--> Robot TX thread -> set_joint
          |
          +--> MuJoCo sim thread -> target_q_sim

Robot state stream -> Robot RX thread -> SharedStateBus
MuJoCo qpos/dq     -> MuJoCo worker   -> SharedStateBus

GUI 50Hz timer -> snapshot() -> table / status / plot
```

## Important simplifications

- no remote replay CSV upload
- no remote replay cursor
- no remote replay start/stop/seek/step/prev
- no second playback loop in MuJoCo
- robot and MuJoCo both follow the same local target owned by the engine

## Why CLI-first

The CLI uses the same engine as the GUI, so replay logic can be validated before opening the Qt UI.
