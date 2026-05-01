"""Qt-facing arm bridge that dispatches ROS 2 actions over paramiko SSH."""
from __future__ import annotations

import logging
import math
from typing import Optional

import yaml
from PySide6.QtCore import QObject, QThread, Signal

from services._ssh_session import SshSession

logger = logging.getLogger(__name__)

# --- Validation constants (cheap safety nets for the arm action API) ---
N_JOINTS: int = 6                 # SO-101 is a 6-DOF arm
MAX_REACH_M: float = 0.6          # generous outer bound (true reach ~0.4 m)
# > this and the caller probably sent degrees
MAX_JOINT_RAD: float = 2.0 * math.pi
MAX_TILT_RAD: float = math.pi     # |pitch|, |roll| > π is almost certainly wrong


class _ActionWorker(QThread):
    """Owns one paramiko channel and parses one ``ros2 action send_goal`` invocation."""

    feedback_emitted = Signal(float, float)        # elapsed_time, current_error
    move_completed_emitted = Signal(bool, float)   # success, final_error

    def __init__(self, action_name: str, payload_yaml: str,
                 parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._action_name = action_name
        self._payload_yaml = payload_yaml
        self._channel = None  # paramiko.Channel | None

    def run(self) -> None:
        """Run the action, parse YAML blocks from stdout, emit feedback/result."""
        if self._action_name == "pose":
            topic = "/move_pose"
            type_str = "poker_interfaces/action/MovePose"
        elif self._action_name == "joints":
            topic = "/move_joints"
            type_str = "poker_interfaces/action/MoveJoints"
        else:
            logger.warning("_ActionWorker: unknown action_name=%r", self._action_name)
            self.move_completed_emitted.emit(False, -1.0)
            return

        cmd = (
            f'ros2 action send_goal --feedback {topic} {type_str} '
            f'"{self._payload_yaml}"'
        )

        try:
            session = SshSession()
            self._channel = session.open_channel(cmd)
        except Exception as exc:
            logger.warning("_ActionWorker: open_channel failed: %s", exc)
            self.move_completed_emitted.emit(False, -1.0)
            return

        buffer = ""
        completed = False
        try:
            while True:
                chunk = self._channel.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode(errors="replace")
                # ros2 action send_goal --feedback emits YAML documents separated
                # by '---' lines; split, parse complete blocks, keep the trailing
                # partial in the buffer for the next read.
                while "\n---\n" in buffer or buffer.startswith("---\n"):
                    if buffer.startswith("---\n"):
                        block = ""
                        buffer = buffer[len("---\n"):]
                    else:
                        block, _, buffer = buffer.partition("\n---\n")
                    block = block.strip()
                    if not block:
                        continue
                    parsed = yaml.safe_load(block)
                    if not isinstance(parsed, dict):
                        continue
                    if "feedback" in parsed:
                        fb = parsed["feedback"] or {}
                        self.feedback_emitted.emit(
                            float(fb.get("elapsed_time", 0.0)),
                            float(fb.get("current_error", 0.0)),
                        )
                    elif "result" in parsed:
                        res = parsed["result"] or {}
                        self.move_completed_emitted.emit(
                            bool(res.get("success", False)),
                            float(res.get("final_error", -1.0)),
                        )
                        completed = True
                        break
                if completed:
                    break
        except Exception as exc:
            logger.warning("_ActionWorker: read/parse failed: %s", exc)
            self.move_completed_emitted.emit(False, -1.0)
            completed = True

        # CLI may close stdout without a final --- separator after result block.
        tail = buffer.strip()
        if tail and not completed:
            try:
                parsed = yaml.safe_load(tail)
                if isinstance(parsed, dict) and "result" in parsed:
                    res = parsed["result"] or {}
                    self.move_completed_emitted.emit(
                        bool(res.get("success", False)),
                        float(res.get("final_error", -1.0)),
                    )
                    completed = True
            except Exception:
                pass

        if not completed:
            # Channel closed before a result block arrived.
            self.move_completed_emitted.emit(False, -1.0)

        try:
            if self._channel is not None:
                self._channel.close()
        except Exception:
            pass
        self._channel = None

    def cancel(self) -> None:
        """Close the channel from another thread to abort the read loop."""
        ch = self._channel
        if ch is None:
            return
        try:
            ch.close()
        except Exception:
            pass


class ArmRosBridge(QObject):
    """Qt-compatible bridge that dispatches arm action goals over SSH."""

    # Qt Signals (for UI binding)
    connection_changed = Signal(bool)     # SSH session up/down
    move_started = Signal(str)            # 'pose' or 'joints'
    move_feedback = Signal(float, float)  # elapsed_time, current_error
    move_completed = Signal(bool, float)  # success, final_error

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ArmRosBridge, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__()
        self._initialized = True

        self._session = SshSession()
        self._session.connection_changed.connect(self.connection_changed)
        self._active_worker: Optional[_ActionWorker] = None

    @property
    def is_available(self) -> bool:
        """True while the underlying SSH session is connected."""
        return self._session.is_connected

    # --- Validation helpers ----------------------------------------------

    @staticmethod
    def _validate_pose(x: float, y: float, z: float,
                       pitch: float, roll: float, duration: float) -> Optional[str]:
        """Return None if the pose is plausible, else a neutral diagnostic string.

        Diagnostics are intentionally terse for autonomous logs on the Pi.
        """
        if not all(isinstance(v, (int, float)) for v in (x, y, z, pitch, roll, duration)):
            return "non-numeric pose component"
        if not all(math.isfinite(v) for v in (x, y, z, pitch, roll, duration)):
            return "non-finite pose component"
        if duration <= 0.0:
            return "non-positive duration"
        reach = math.sqrt(x * x + y * y + z * z)
        if reach > MAX_REACH_M:
            return f"reach {reach:.3f}m > MAX_REACH_M ({MAX_REACH_M:.2f})"
        if abs(pitch) > MAX_TILT_RAD or abs(roll) > MAX_TILT_RAD:
            return f"|pitch|={abs(pitch):.2f} or |roll|={abs(roll):.2f} > MAX_TILT_RAD"
        return None

    @staticmethod
    def _validate_joints(joints, duration: float) -> Optional[str]:
        """Return None if the joint vector is plausible, else a diagnostic string."""
        try:
            joints_list = list(joints)
        except TypeError:
            return "joints not iterable"
        if len(joints_list) != N_JOINTS:
            return f"len(joints)={len(joints_list)} != {N_JOINTS}"
        if not all(isinstance(j, (int, float)) for j in joints_list):
            return "non-numeric joint value"
        if not all(math.isfinite(j) for j in joints_list):
            return "non-finite joint value"
        if not math.isfinite(duration):
            return "non-finite duration"
        if duration <= 0.0:
            return "non-positive duration"
        for i, j in enumerate(joints_list):
            if abs(j) > MAX_JOINT_RAD:
                logger.warning(
                    "ArmRosBridge: joint[%d]=%.3f exceeds MAX_JOINT_RAD=%.2f",
                    i, j, MAX_JOINT_RAD)
        return None

    # --- Public API: Action-based (goal/feedback/result) ---

    def move_pose(self, x: float, y: float, z: float,
                  pitch: float, roll: float, duration: float) -> None:
        """Dispatch a MovePose goal over SSH. Non-blocking; results via signals."""
        err = self._validate_pose(x, y, z, pitch, roll, duration)
        if err is not None:
            logger.warning("ArmRosBridge.move_pose rejected: %s", err)
            self.move_completed.emit(False, -1.0)
            return

        payload = (
            f"{{x: {float(x)}, y: {float(y)}, z: {float(z)}, "
            f"pitch: {float(pitch)}, roll: {float(roll)}, "
            f"duration: {float(duration)}}}"
        )
        self.move_started.emit("pose")
        self._spawn_worker("pose", payload)

    def move_joints(self, joints, duration: float) -> None:
        """Dispatch a MoveJoints goal over SSH. Non-blocking; results via signals."""
        err = self._validate_joints(joints, duration)
        if err is not None:
            logger.warning("ArmRosBridge.move_joints rejected: %s", err)
            self.move_completed.emit(False, -1.0)
            return

        joints_list = [float(j) for j in joints]
        joints_str = ", ".join(repr(j) for j in joints_list)
        payload = f"{{joints: [{joints_str}], duration: {float(duration)}}}"
        self.move_started.emit("joints")
        self._spawn_worker("joints", payload)

    def cancel_move(self) -> None:
        """Abort the active worker, if any. Triggers a failure result via signals."""
        worker = self._active_worker
        if worker is None:
            return
        if worker.isRunning():
            worker.cancel()

    def shutdown(self) -> None:
        """Cancel any in-flight worker and wait for it. SSH session is owned elsewhere."""
        worker = self._active_worker
        self._active_worker = None
        if worker is None:
            return
        if worker.isRunning():
            worker.cancel()
            # Bound the wait so a stuck SSH connect cannot hang teardown.
            worker.wait(2000)

    # --- Internal -------------------------------------------------------

    def _spawn_worker(self, action_name: str, payload_yaml: str) -> None:
        # Drop any prior worker before starting a new one so two threads do not
        # race over the singleton SSH transport.
        prior = self._active_worker
        if prior is not None and prior.isRunning():
            prior.cancel()
            prior.wait(2000)

        worker = _ActionWorker(action_name=action_name, payload_yaml=payload_yaml)
        worker.feedback_emitted.connect(self.move_feedback)
        worker.move_completed_emitted.connect(self.move_completed)
        self._active_worker = worker
        worker.start()
