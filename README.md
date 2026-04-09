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
```
