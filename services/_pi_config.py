import os
import shlex


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


PI_HOST = os.getenv("PI_HOST", "pi.local")
PI_USER = os.getenv("PI_USER", "pi")
PI_KEY_PATH = os.getenv("PI_KEY_PATH", None)
PI_PASS = os.getenv("PI_PASS", None)
WS_PATH = os.getenv("WS_PATH", "~/poker_arm_ws")
ROS_SETUP = os.getenv("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
PUMP_SETTLE_S = _env_float("PUMP_SETTLE_S", 0.05)


def build_remote_command(cmd: str) -> str:
    """Wrap cmd in a bash -lc invocation that sources ROS and the workspace before running it."""
    inner = f"source {ROS_SETUP} && source {WS_PATH}/install/setup.bash && {cmd}"
    return f"bash -lc {shlex.quote(inner)}"
