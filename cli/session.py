from __future__ import annotations

import argparse
import shlex

from cli.printers import print_joint_table, print_logs, print_status
from replay_core.replay_engine import ReplayEngine

HELP = """
Commands:
  help
  load_csv <path>
  load_mujoco <xml_path>
  connect <host> [cmd_port] [state_port]
  ping
  init [seconds]
  enable
  disable
  start [frame]
  stop
  seek <frame>
  step
  prev
  status
  table
  logs
  quit
""".strip()


def main() -> None:
    parser = argparse.ArgumentParser(description='CLI for the local replay debugger')
    parser.add_argument('--csv', help='optional CSV to preload')
    parser.add_argument('--xml', help='optional MuJoCo XML to preload')
    args = parser.parse_args()

    engine = ReplayEngine()
    try:
        if args.csv:
            engine.load_csv(args.csv)
        if args.xml:
            engine.load_mujoco(args.xml, start_viewer=True)

        print(HELP)
        while True:
            try:
                raw = input('dog-replay> ').strip()
            except EOFError:
                break
            if not raw:
                continue
            parts = shlex.split(raw)
            cmd = parts[0].lower()
            try:
                if cmd == 'help':
                    print(HELP)
                elif cmd == 'load_csv' and len(parts) >= 2:
                    print('OK' if engine.load_csv(parts[1]) else 'FAIL')
                elif cmd == 'load_mujoco' and len(parts) >= 2:
                    print('OK' if engine.load_mujoco(parts[1], start_viewer=True) else 'FAIL')
                elif cmd == 'connect' and len(parts) >= 2:
                    host = parts[1]
                    cmd_port = None if len(parts) < 3 else int(parts[2])
                    state_port = None if len(parts) < 4 else int(parts[3])
                    print('OK' if engine.connect_robot(host, cmd_port, state_port) else 'FAIL')
                elif cmd == 'ping':
                    print('OK' if engine.robot_ping() else 'FAIL')
                elif cmd == 'init':
                    seconds = 2.5 if len(parts) < 2 else float(parts[1])
                    print(engine.robot_init(seconds))
                elif cmd == 'enable':
                    print(engine.robot_enable())
                elif cmd == 'disable':
                    print(engine.robot_disable())
                elif cmd == 'start':
                    frame = None if len(parts) < 2 else int(parts[1])
                    print('OK' if engine.start(frame) else 'FAIL')
                elif cmd == 'stop':
                    engine.stop(); print('OK')
                elif cmd == 'seek' and len(parts) >= 2:
                    print('OK' if engine.seek(int(parts[1])) else 'FAIL')
                elif cmd == 'step':
                    print('OK' if engine.step() else 'FAIL')
                elif cmd == 'prev':
                    print('OK' if engine.prev() else 'FAIL')
                elif cmd == 'status':
                    print_status(engine.get_snapshot())
                elif cmd == 'table':
                    print_joint_table(engine.get_snapshot())
                elif cmd == 'logs':
                    print_logs(engine.get_snapshot().log_lines)
                elif cmd in {'quit', 'exit'}:
                    break
                else:
                    print('Unknown command')
            except Exception as exc:
                print(f'ERROR: {exc}')
    finally:
        engine.close()
