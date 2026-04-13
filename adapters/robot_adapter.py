from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional

import numpy as np

from replay_core.constants import NUM_JOINTS
from replay_core.metrics import RateTracker
from replay_core.sync_bus import SharedStateBus
from replay_core.types import BackendState


class RobotAdapter:
    def __init__(self, bus: SharedStateBus, timeout_s: float = 3.0, tx_period_s: float = 0.02):
        self.bus = bus
        self.timeout_s = float(timeout_s)
        self.tx_period_s = float(tx_period_s)

        self.host = ''
        self.cmd_port = 0
        self.state_port = 0

        self._cmd_sock: Optional[socket.socket] = None
        self._state_sock: Optional[socket.socket] = None
        self._cmd_lock = threading.Lock()
        self._stop = threading.Event()
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_rate = RateTracker()
        self._rx_rate = RateTracker()
        self._tx_timeout_streak = 0
        self._tx_reconnect_threshold = 3

    @property
    def connected(self) -> bool:
        return self._cmd_sock is not None and self._state_sock is not None

    def connect(self, host: str, cmd_port: int, state_port: int) -> bool:
        self.disconnect()
        self.host = str(host)
        self.cmd_port = int(cmd_port)
        self.state_port = int(state_port)
        try:
            self._cmd_sock = socket.create_connection((self.host, self.cmd_port), timeout=self.timeout_s)
            self._cmd_sock.settimeout(self.timeout_s)
            self._state_sock = socket.create_connection((self.host, self.state_port), timeout=self.timeout_s)
            self._state_sock.settimeout(0.5)
        except OSError as exc:
            self.bus.log(f'Robot connect failed: {exc}')
            self.disconnect()
            return False

        self.bus.set_robot_connected(True)
        self.bus.log(f'Robot connected to {self.host}:{self.cmd_port}/{self.state_port}')
        self._tx_timeout_streak = 0
        self._stop.clear()
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread.start()
        self._rx_thread.start()
        return True

    def disconnect(self) -> None:
        self._stop.set()
        for thread in (self._tx_thread, self._rx_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=1.0)
        self._tx_thread = None
        self._rx_thread = None
        for sock in (self._cmd_sock, self._state_sock):
            if sock is not None:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    sock.close()
                except OSError:
                    pass
        self._cmd_sock = None
        self._state_sock = None
        self._tx_timeout_streak = 0
        self.bus.set_robot_connected(False)
        self.bus.set_robot_rates(0.0, 0.0)

    def _send_command(self, command: str, timeout: Optional[float] = None) -> Dict[str, Any]:
        payload = (command.strip() + '\n').encode('utf-8')
        with self._cmd_lock:
            sock = self._cmd_sock
            if sock is None:
                raise RuntimeError('robot command socket is not connected')
            old_timeout = sock.gettimeout()
            if timeout is not None:
                sock.settimeout(timeout)
            try:
                sock.sendall(payload)
                buffer = b''
                while b'\n' not in buffer:
                    chunk = sock.recv(4096)
                    if not chunk:
                        raise RuntimeError('robot command socket closed')
                    buffer += chunk
            finally:
                try:
                    sock.settimeout(old_timeout)
                except OSError:
                    pass
        reply = json.loads(buffer.split(b'\n', 1)[0].decode('utf-8'))
        return reply

    @staticmethod
    def _is_timeout_error(exc: Exception) -> bool:
        if isinstance(exc, (socket.timeout, TimeoutError)):
            return True
        return 'timed out' in str(exc).lower()

    def _reconnect_cmd_socket(self) -> bool:
        if self._stop.is_set():
            return False
        if not self.host or self.cmd_port <= 0:
            self.bus.log('Robot TX reconnect skipped: host/port not set')
            return False
        try:
            new_sock = socket.create_connection((self.host, self.cmd_port), timeout=self.timeout_s)
            new_sock.settimeout(self.timeout_s)
        except OSError as exc:
            self.bus.log(f'Robot TX reconnect failed: {exc}')
            return False

        old_sock: Optional[socket.socket]
        with self._cmd_lock:
            old_sock = self._cmd_sock
            self._cmd_sock = new_sock

        if old_sock is not None:
            try:
                old_sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                old_sock.close()
            except OSError:
                pass
        return True

    def ping(self) -> bool:
        try:
            reply = self._send_command('ping')
            ok = bool(reply.get('ok', False))
            self.bus.log('Robot ping OK' if ok else f'Robot ping failed: {reply}')
            return ok
        except Exception as exc:
            self.bus.log(f'Robot ping failed: {exc}')
            return False

    def init(self, duration_s: float = 2.5) -> Dict[str, Any]:
        try:
            reply = self._send_command(f'init {float(duration_s):.3f}', timeout=max(self.timeout_s, duration_s + 4.0))
            self.bus.log(f'Robot init reply: {reply}')
            return reply
        except Exception as exc:
            self.bus.log(f'Robot init failed: {exc}')
            return {'ok': False, 'msg': str(exc)}

    def disable(self) -> Dict[str, Any]:
        try:
            reply = self._send_command('disable')
            self.bus.log(f'Robot disable reply: {reply}')
            return reply
        except Exception as exc:
            self.bus.log(f'Robot disable failed: {exc}')
            return {'ok': False, 'msg': str(exc)}

    def set_mit_param(self, kp: float, kd: float, vel_limit: float, torque_limit: float) -> Dict[str, Any]:
        try:
            reply = self._send_command(
                f'set_mit_param {float(kp):.6f} {float(kd):.6f} {float(vel_limit):.6f} {float(torque_limit):.6f}'
            )
            self.bus.log(f'Robot MIT param reply: {reply}')
            return reply
        except Exception as exc:
            self.bus.log(f'Robot MIT param failed: {exc}')
            return {'ok': False, 'msg': str(exc)}

    def set_zero_joint(self, joint_idx: int) -> Dict[str, Any]:
        try:
            reply = self._send_command(f'setzero {int(joint_idx)}')
            self.bus.log(f'Robot setzero reply: {reply}')
            return reply
        except Exception as exc:
            self.bus.log(f'Robot setzero failed: {exc}')
            return {'ok': False, 'msg': str(exc)}

    def _set_joint(self, targets_rad: np.ndarray) -> None:
        values = ' '.join(f'{float(v):.6f}' for v in targets_rad.tolist())
        reply = self._send_command(f'set_joint {values}')
        if not reply.get('ok', False):
            raise RuntimeError(str(reply))

    def _tx_loop(self) -> None:
        last_robot_seq = -1
        while not self._stop.is_set():
            if self._cmd_sock is None:
                break
            try:
                sent = False
                if self.bus.is_playing():
                    target = self.bus.get_target()
                    self._set_joint(target)
                    sent = True
                else:
                    current_seq = self.bus.get_robot_command_seq()
                    if current_seq != last_robot_seq:
                        target = self.bus.get_target()
                        self._set_joint(target)
                        last_robot_seq = current_seq
                        sent = True
                if sent:
                    self._tx_rate.tick()
                    self._tx_timeout_streak = 0
                self.bus.set_robot_rates(self._tx_rate.value(), self._rx_rate.value())
            except Exception as exc:
                if self._is_timeout_error(exc):
                    self._tx_timeout_streak += 1
                    self.bus.log(f'Robot TX timeout ({self._tx_timeout_streak}): {exc}')
                    if self._tx_timeout_streak >= self._tx_reconnect_threshold:
                        if self._reconnect_cmd_socket():
                            self.bus.log('Robot TX command socket reconnected')
                            self._tx_timeout_streak = 0
                        else:
                            self.bus.log('Robot TX command socket reconnect failed')
                else:
                    self._tx_timeout_streak = 0
                    self.bus.log(f'Robot TX error: {exc}')
                time.sleep(0.2)
            time.sleep(self.tx_period_s)

    def _rx_loop(self) -> None:
        if self._state_sock is None:
            return
        buffer = b''
        while not self._stop.is_set():
            try:
                chunk = self._state_sock.recv(4096)
                if not chunk:
                    raise RuntimeError('robot state socket closed')
                buffer += chunk
            except socket.timeout:
                self.bus.set_robot_rates(self._tx_rate.value(), self._rx_rate.value())
                continue
            except Exception as exc:
                self.bus.log(f'Robot RX error: {exc}')
                break

            while b'\n' in buffer:
                raw, buffer = buffer.split(b'\n', 1)
                if not raw.strip():
                    continue
                try:
                    text = raw.decode('utf-8')
                    payload = json.loads(text)
                    state = payload.get('state', payload)
                    now = time.monotonic()

                    positions = np.asarray(state.get('joint_positions', []), dtype=np.float64)
                    if positions.shape[0] != NUM_JOINTS:
                        continue
                    torques = np.asarray(state.get('joint_torques', [0.0] * NUM_JOINTS), dtype=np.float64)
                    if torques.shape[0] != NUM_JOINTS:
                        torques = np.zeros(NUM_JOINTS, dtype=np.float64)
                    target_joint_positions = np.asarray(state.get('target_joint_positions', [0.0] * NUM_JOINTS), dtype=np.float64)
                    if target_joint_positions.shape[0] != NUM_JOINTS:
                        target_joint_positions = np.zeros(NUM_JOINTS, dtype=np.float64)
                    last_sent_joint_positions = np.asarray(state.get('last_sent_joint_positions', [0.0] * NUM_JOINTS), dtype=np.float64)
                    if last_sent_joint_positions.shape[0] != NUM_JOINTS:
                        last_sent_joint_positions = np.zeros(NUM_JOINTS, dtype=np.float64)

                    backend_state = BackendState(
                        ok=bool(state.get('ok', payload.get('ok', True))),
                        enabled=bool(state.get('enabled', False)),
                        worker_started=bool(state.get('worker_started', False)),
                        busy=bool(state.get('busy', False)),
                        init_in_progress=bool(state.get('init_in_progress', False)),
                        queue_size=int(state.get('queue_size', 0)),
                        kp=None if state.get('kp') is None else float(state.get('kp')),
                        kd=None if state.get('kd') is None else float(state.get('kd')),
                        joint_positions=positions,
                        joint_torques=torques,
                        target_joint_positions=target_joint_positions,
                        last_sent_joint_positions=last_sent_joint_positions,
                        last_error=str(state.get('last_error', '')),
                        raw_json=text,
                        timestamp_sec=now,
                    )
                    self.bus.update_backend_state(backend_state)
                    self.bus.update_robot_state(positions, np.zeros(NUM_JOINTS, dtype=np.float64), np.zeros(NUM_JOINTS, dtype=np.float64), now)
                    self._rx_rate.tick()
                    self.bus.set_robot_rates(self._tx_rate.value(), self._rx_rate.value())
                except Exception as exc:
                    self.bus.log(f'Robot RX parse error: {exc}')
