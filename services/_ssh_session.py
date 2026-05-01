"""Qt singleton wrapping a paramiko SSHClient shared by remote bridges.

Connection is lazy: the first call to ``exec_command`` or ``open_channel``
triggers ``_ensure_connected``. Construction is cheap so the singleton is
safe to import before the GUI exists and before the Pi is reachable.
"""
from __future__ import annotations

import logging
import threading

import paramiko
from PySide6.QtCore import QObject, Signal

from services._pi_config import (
    PI_HOST,
    PI_KEY_PATH,
    PI_PASS,
    PI_USER,
    build_remote_command,
)

logger = logging.getLogger(__name__)

# Bridges spawn QThreads that share this singleton; lock prevents duplicate construction.
_singleton_lock = threading.Lock()


class SshSession(QObject):
    """Lazy-connect singleton wrapping ``paramiko.SSHClient``."""

    connection_changed = Signal(bool)

    _instance = None

    def __new__(cls):
        with _singleton_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        super().__init__()
        self._initialized = True
        # Lazy: do NOT connect here. Existing bridges (ArmRosBridge,
        # TableIoBridge) connect in __init__, but an SSH connect blocks
        # on a network round-trip and pre-empting the GUI for that is
        # unacceptable. The first exec/open call kicks the connection.
        self._client: paramiko.SSHClient | None = None
        self._connected: bool = False
        self._connect_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        """Return True if the underlying SSH client is live."""
        return self._connected

    def _ensure_connected(self) -> None:
        """Open the SSH connection if not already established."""
        with self._connect_lock:
            if self._client is not None and self._connected:
                return
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            try:
                client.connect(
                    hostname=PI_HOST,
                    username=PI_USER,
                    key_filename=PI_KEY_PATH,
                    password=PI_PASS,
                    look_for_keys=True,
                    allow_agent=True,
                    timeout=10,
                )
            except Exception as exc:
                logger.warning("SshSession: connect to %s@%s failed: %s",
                               PI_USER, PI_HOST, exc)
                self._client = None
                self._connected = False
                self.connection_changed.emit(False)
                raise
            self._client = client
            self._connected = True
            self.connection_changed.emit(True)
            logger.info("SshSession: connected to %s@%s", PI_USER, PI_HOST)

    def exec_command(self, cmd: str, timeout: float | None = None) -> tuple[int, str, str]:
        """Run ``cmd`` on the Pi (wrapped in the ROS-sourcing bash) and return (rc, stdout, stderr)."""
        wrapped = build_remote_command(cmd)
        self._ensure_connected()
        assert self._client is not None
        _stdin, stdout, stderr = self._client.exec_command(wrapped, timeout=timeout)
        out_bytes = stdout.read()
        err_bytes = stderr.read()
        rc = stdout.channel.recv_exit_status()
        return rc, out_bytes.decode(errors="replace"), err_bytes.decode(errors="replace")

    def open_channel(self, cmd: str) -> paramiko.Channel:
        """Open a streaming channel running ``cmd`` (wrapped). Caller closes it."""
        wrapped = build_remote_command(cmd)
        self._ensure_connected()
        assert self._client is not None
        transport = self._client.get_transport()
        if transport is None:
            raise paramiko.SSHException("SshSession: transport unavailable after connect")
        channel = transport.open_session()
        channel.set_combine_stderr(True)
        channel.exec_command(wrapped)
        return channel

    def shutdown(self) -> None:
        """Close the client and reset singleton state. Safe to call repeatedly."""
        was_connected = self._connected
        if self._client is not None:
            try:
                self._client.close()
            except Exception:
                pass
        self._client = None
        self._connected = False
        if was_connected:
            self.connection_changed.emit(False)
        type(self)._instance = None
        self._initialized = False
