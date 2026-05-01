"""ROS 2 bridge for the table-side button + suction pump.

Mirrors services/arm_ros_bridge.py.

- Subscribes to ``/button_count`` (published by
  ``src/poker_control/poker_control/pump_test.py``) and emits
  ``turn_advance_requested(seat: int)`` so MainWindow can call
  ``GameState.next_turn()``.
- Exposes ``set_pump(state: bool)`` which publishes a single ``Int32(1)``
  or ``Int32(0)`` on ``/pump_control``. After the ``pump_duration_s``
  GPIO settle delay, emits ``pump_state_set(state)`` so callers know the
  hardware-side toggle has been registered.

Wired by MainWindow to ``ArmChoreographer.pump_requested`` (consumes,
calls ``set_pump``) and ``ArmChoreographer._on_pump_done`` (produces,
receives ``pump_state_set``).

Singleton — shares the rclpy.init guard with arm_ros_bridge.
"""
from __future__ import annotations
import logging
from typing import Optional

from PySide6.QtCore import QObject, Signal, QTimer

logger = logging.getLogger(__name__)

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    Node = object


class _TableIoNode(Node):
    """Low-level ROS 2 node. Only instantiated when rclpy is available.

    The ``pump_duration_s`` parameter is the post-publish GPIO settle
    delay (seconds) used by the Qt bridge before emitting
    ``pump_state_set``. It is no longer a pulse duration; the bridge
    publishes a single Int32(1)/Int32(0) without auto-off.
    """

    def __init__(self, on_button_pressed):
        super().__init__('table_io_bridge')
        self.declare_parameter('pump_duration_s', 0.05)
        self.pump_duration_s = self.get_parameter('pump_duration_s').value

        self._on_button_pressed = on_button_pressed
        self.create_subscription(Int32, '/button_count', self._on_button_count, 10)
        self.pump_pub = self.create_publisher(Int32, '/pump_control', 10)

    def _on_button_count(self, msg: Int32):
        self._on_button_pressed(msg.data)

    def publish_pump(self, on: bool):
        msg = Int32()
        msg.data = 1 if on else 0
        self.pump_pub.publish(msg)


class TableIoBridge(QObject):
    """Singleton Qt-side bridge."""
    _instance = None

    turn_advance_requested = Signal(int)
    pump_state_set = Signal(bool)

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
        self._node: Optional[_TableIoNode] = None
        self._spin_timer: Optional[QTimer] = None
        self.is_available = False

        # Persistent settle timer + latched state: a second set_pump within the
        # window cancels the pending emit so pump_state_set fires once with the
        # latest requested value.
        self._pending_pump_state: bool = False
        self._pump_settle_timer = QTimer(self)
        self._pump_settle_timer.setSingleShot(True)
        self._pump_settle_timer.timeout.connect(self._emit_pump_state_set)

        if not _ROS_AVAILABLE:
            logger.info("TableIoBridge: rclpy not available, running as stub.")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = _TableIoNode(on_button_pressed=self._handle_button)
            self._spin_timer = QTimer(self)
            self._spin_timer.timeout.connect(self._spin_once)
            self._spin_timer.start(10)  # 100Hz
            self.is_available = True
            logger.info("TableIoBridge: connected to ROS 2.")
        except Exception as exc:
            logger.error(f"TableIoBridge: failed to init ROS 2: {exc}")
            self._node = None

    def _spin_once(self):
        if self._node is not None:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.0)
            except Exception as exc:
                logger.warning(f"TableIoBridge: spin_once error: {exc}")

    def _handle_button(self, seat: int):
        self.turn_advance_requested.emit(seat)

    def set_pump(self, on: bool):
        """Publish a single Int32(on) on /pump_control and emit
        ``pump_state_set(on)`` once the GPIO settle delay has elapsed."""
        if not self.is_available or self._node is None:
            return
        self._node.publish_pump(on)
        # Settle delay lets the GPIO line stabilise before the choreographer
        # treats the toggle as committed; no second publish is sent.
        self._pump_settle_timer.stop()
        self._pending_pump_state = on
        self._pump_settle_timer.start(int(self._node.pump_duration_s * 1000))
        logger.info(f"TableIoBridge: pump set to {on}")

    def _emit_pump_state_set(self):
        self.pump_state_set.emit(self._pending_pump_state)

    def shutdown(self):
        if self._spin_timer is not None:
            self._spin_timer.stop()
        if self._node is not None:
            self._node.destroy_node()
