"""Table-side button + suction pump bridge over paramiko SSH.

The Qt singleton spawns a persistent ``_ButtonCountWorker`` QThread that
streams ``ros2 topic echo /button_count`` over an SSH channel and emits
``turn_advance_requested``, and on each ``set_pump`` call spawns a
one-shot ``_PumpWorker`` QThread that publishes a single ``Int32`` on
``/pump_control`` then triggers a short settle delay before emitting
``pump_state_set``.
"""
from __future__ import annotations
import logging
from typing import Optional

import yaml
from PySide6.QtCore import QObject, QThread, Signal, QTimer

from services._ssh_session import SshSession
from services._pi_config import PUMP_SETTLE_S

logger = logging.getLogger(__name__)


class _ButtonCountWorker(QThread):
    """Persistent SSH-backed reader for ``/button_count`` Int32 messages."""

    seat_emitted = Signal(int)

    def __init__(self, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._channel = None  # paramiko.Channel | None

    def run(self):
        """Stream YAML blocks from ros2 topic echo and emit each seat value."""
        try:
            session = SshSession()
            self._channel = session.open_channel(
                "ros2 topic echo /button_count std_msgs/msg/Int32"
            )
        except Exception as exc:
            logger.warning("_ButtonCountWorker: open_channel failed: %s", exc)
            return

        buffer = ""
        try:
            while not self.isInterruptionRequested():
                if self._channel is None or self._channel.closed:
                    break
                # recv blocks; keep the loop responsive to requestInterruption
                # by using a small read with a short timeout.
                if not self._channel.recv_ready():
                    if self._channel.exit_status_ready():
                        break
                    self.msleep(50)
                    continue
                chunk = self._channel.recv(4096)
                if not chunk:
                    break
                buffer += chunk.decode(errors="replace")
                while "---" in buffer:
                    block, _, buffer = buffer.partition("---")
                    block = block.strip()
                    if not block:
                        continue
                    try:
                        parsed = yaml.safe_load(block)
                    except Exception as exc:
                        logger.warning(
                            "_ButtonCountWorker: yaml parse failed: %s", exc)
                        continue
                    if isinstance(parsed, dict) and "data" in parsed:
                        try:
                            self.seat_emitted.emit(int(parsed["data"]))
                        except (TypeError, ValueError) as exc:
                            logger.warning(
                                "_ButtonCountWorker: bad data field: %s", exc)
        except Exception as exc:
            logger.warning("_ButtonCountWorker: read loop error: %s", exc)
        finally:
            try:
                if self._channel is not None:
                    self._channel.close()
            except Exception:
                pass
            self._channel = None

    def stop(self):
        """Request interruption and close the SSH channel."""
        self.requestInterruption()
        try:
            if self._channel is not None:
                self._channel.close()
        except Exception:
            pass


class _PumpWorker(QThread):
    """One-shot publisher for ``/pump_control`` (single Int32, no auto-off)."""

    publish_complete = Signal(bool)

    def __init__(self, state: bool, parent: Optional[QObject] = None):
        super().__init__(parent)
        self._state = state

    def run(self):
        """Publish a single Int32 message, then signal completion."""
        payload = "{data: 1}" if self._state else "{data: 0}"
        cmd = f'ros2 topic pub --once /pump_control std_msgs/msg/Int32 "{payload}"'
        try:
            session = SshSession()
            rc, _stdout, stderr = session.exec_command(cmd)
            if rc != 0:
                logger.warning(
                    "_PumpWorker: rc=%s state=%s stderr=%s",
                    rc, self._state, stderr.strip())
        except Exception as exc:
            logger.warning("_PumpWorker: exec_command failed: %s", exc)
        # Always emit so the choreographer's pump step does not stall.
        self.publish_complete.emit(self._state)


class TableIoBridge(QObject):
    """Singleton Qt-side bridge for the table button + pump over SSH."""

    connection_changed = Signal(bool)
    turn_advance_requested = Signal(int)
    pump_state_set = Signal(bool)

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__()
        self._initialized = True

        # Proxy SshSession's connection_changed to our local signal.
        session = SshSession()
        session.connection_changed.connect(self.connection_changed)

        # Persistent settle timer + latched state: a second set_pump within the
        # window cancels the pending emit so pump_state_set fires once with the
        # latest requested value.
        self._pending_pump_state: bool = False
        self._pump_settle_timer = QTimer(self)
        self._pump_settle_timer.setSingleShot(True)
        self._pump_settle_timer.timeout.connect(self._emit_pump_state_set)

        self._button_worker = _ButtonCountWorker(self)
        self._button_worker.seat_emitted.connect(self.turn_advance_requested)
        self._button_worker.start()

    def set_pump(self, on: bool):
        """Publish Int32(on) on /pump_control and emit pump_state_set after settle."""
        # Cancel any in-flight settle so the latest requested state wins.
        self._pump_settle_timer.stop()
        self._pending_pump_state = on

        worker = _PumpWorker(state=on, parent=self)
        worker.publish_complete.connect(self._on_pump_publish_complete)
        worker.start()
        logger.info("TableIoBridge: pump set to %s", on)

    def _on_pump_publish_complete(self, _state: bool):
        self._pump_settle_timer.start(int(PUMP_SETTLE_S * 1000))

    def _emit_pump_state_set(self):
        self.pump_state_set.emit(self._pending_pump_state)

    def shutdown(self):
        """Stop the persistent button worker and pending settle timer."""
        self._pump_settle_timer.stop()
        try:
            self._button_worker.stop()
            self._button_worker.wait(1000)
        except Exception:
            pass
