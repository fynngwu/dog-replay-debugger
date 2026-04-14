from __future__ import annotations

CMD_REQUEST_MODE = "request_mode"
CMD_TARGET = "target"
CMD_GET_MODE = "get_mode"
CMD_GET_JOINTS = "get_joints"
CMD_GET_IMU = "get_imu"
CMD_GET_ALL = "get_all"

MODES = ("init", "execute", "policy", "stop")


def make_request_mode(mode: str) -> str:
    return f"{CMD_REQUEST_MODE} {mode}"


def make_target(joints: list[float]) -> str:
    vals = " ".join(f"{v:.6f}" for v in joints)
    return f"{CMD_TARGET} {vals}"


def make_get_mode() -> str:
    return CMD_GET_MODE


def make_get_joints() -> str:
    return CMD_GET_JOINTS


def make_get_imu() -> str:
    return CMD_GET_IMU


def make_get_all() -> str:
    return CMD_GET_ALL


def parse_reply(raw: dict) -> tuple[bool, str, dict | None]:
    ok = bool(raw.get("ok", False))
    msg = raw.get("msg", "")
    data = raw.get("data")
    return ok, msg, data
