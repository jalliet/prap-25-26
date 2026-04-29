"""
ROS 2 bridge for the poker robot arm controller.

All ROS 2 imports are optional. When rclpy or poker_interfaces are not
installed, the bridge silently degrades to no-op stubs so the main
application continues to function for camera and poker logic only.
"""
from __future__ import annotations
import logging
from typing import Optional, Callable

from PySide6.QtCore import QObject, Signal, QTimer

logger = logging.getLogger(__name__)

# Conditional ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from poker_interfaces.msg import TargetPose, TargetJoints, MotorFeedback
    from poker_interfaces.action import MovePose, MoveJoints
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False
    Node = object  # stub for class inheritance


class _ArmRosNode(Node):
    """Low-level ROS 2 node. Only instantiated when rclpy is available."""

    def __init__(self):
        super().__init__('poker_arm_bridge')

        # Publishers (topic-based streaming/proposals)
        self.pub_target_pose = self.create_publisher(TargetPose, '/target_pose', 10)
        self.pub_target_joints = self.create_publisher(TargetJoints, '/target_joints', 10)

        # Subscriber (telemetry)
        self.sub_motor_feedback = self.create_subscription(
            MotorFeedback, '/motor_feedback', self._on_motor_feedback, 10)

        # Action clients (blocking execution with feedback)
        self.pose_action_client = ActionClient(self, MovePose, '/move_pose')
        self.joints_action_client = ActionClient(self, MoveJoints, '/move_joints')

        # Feedback relay
        self._feedback_callbacks: list[Callable] = []

    def _on_motor_feedback(self, msg):
        """Convert ROS msg to plain dict, invoke registered callbacks."""
        data = {
            'servo_ids': list(msg.servo_ids),
            'positions': list(msg.positions),
            'speeds': list(msg.speeds),
            'loads': list(msg.loads),
            'voltages': list(msg.voltages),
            'temperatures': list(msg.temperatures),
            'feedback_valid': msg.feedback_valid,
        }
        for cb in self._feedback_callbacks:
            cb(data)


class ArmRosBridge(QObject):
    """
    Qt-compatible bridge to the ROS 2 arm controller stack.

    Gracefully degrades to a no-op if rclpy or poker_interfaces
    are not installed. All public methods are safe to call regardless.
    """

    # Qt Signals (for UI binding)
    feedback_received = Signal(dict)
    move_started = Signal(str)            # 'pose' or 'joints'
    move_completed = Signal(bool, float)  # success, final_error
    move_feedback = Signal(float, float)  # current_error, elapsed_time
    connection_changed = Signal(bool)     # ros available True/False

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

        self._ros_available: bool = False
        self._node: Optional[_ArmRosNode] = None
        self._spin_timer: Optional[QTimer] = None
        self._active_goal_handle = None

        self._try_init_ros()

    def _try_init_ros(self):
        """Attempt to import rclpy and init. Sets self._ros_available."""
        if not _ROS_AVAILABLE:
            logger.info("ArmRosBridge: rclpy/poker_interfaces not installed. Arm control disabled.")
            self.connection_changed.emit(False)
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = _ArmRosNode()
            self._node._feedback_callbacks.append(self._relay_feedback)

            self._spin_timer = QTimer()
            self._spin_timer.timeout.connect(self._spin_once)
            self._spin_timer.start(10)  # 100 Hz, matches arm controller branch pattern

            self._ros_available = True
            self.connection_changed.emit(True)
            logger.info("ArmRosBridge: ROS 2 connected.")
        except Exception as e:
            logger.warning(f"ArmRosBridge: ROS 2 init failed ({e}). Arm control disabled.")
            self._ros_available = False
            self.connection_changed.emit(False)

    @property
    def is_available(self) -> bool:
        return self._ros_available

    def _spin_once(self):
        """Pump the ROS 2 event loop from within Qt's event loop."""
        if self._node is None:
            return
        try:
            if rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0)
        except Exception:
            pass

    def _relay_feedback(self, data: dict):
        """Forward motor feedback from ROS callback to Qt signal."""
        self.feedback_received.emit(data)

    # --- Public API: Topic-based (fire-and-forget proposals) ---

    def publish_pose_proposal(self, x: float, y: float, z: float,
                              pitch: float, roll: float, duration: float):
        """Publish a pose proposal on /target_pose. Non-blocking."""
        if not self._ros_available or self._node is None:
            return
        msg = TargetPose()
        msg.x, msg.y, msg.z = x, y, z
        msg.pitch, msg.roll = pitch, roll
        msg.duration = duration
        self._node.pub_target_pose.publish(msg)

    def publish_joint_proposal(self, joints: list, duration: float):
        """Publish a joint proposal on /target_joints. Non-blocking."""
        if not self._ros_available or self._node is None:
            return
        msg = TargetJoints()
        msg.joints = [float(j) for j in joints]
        msg.duration = duration
        self._node.pub_target_joints.publish(msg)

    # --- Public API: Action-based (goal/feedback/result) ---

    def move_pose(self, x: float, y: float, z: float,
                  pitch: float, roll: float, duration: float):
        """
        Send a MovePose action goal. Non-blocking (async).
        Results delivered via move_completed signal.
        Feedback delivered via move_feedback signal.
        """
        if not self._ros_available or self._node is None:
            self.move_completed.emit(False, -1.0)
            return

        client = self._node.pose_action_client
        if not client.wait_for_server(timeout_sec=0.5):
            logger.warning("ArmRosBridge: MovePose action server not available.")
            self.move_completed.emit(False, -1.0)
            return

        goal = MovePose.Goal()
        goal.x, goal.y, goal.z = x, y, z
        goal.pitch, goal.roll = pitch, roll
        goal.duration = duration

        self.move_started.emit('pose')
        future = client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        future.add_done_callback(self._on_goal_accepted)

    def move_joints(self, joints: list, duration: float):
        """
        Send a MoveJoints action goal. Non-blocking (async).
        Results delivered via move_completed signal.
        """
        if not self._ros_available or self._node is None:
            self.move_completed.emit(False, -1.0)
            return

        client = self._node.joints_action_client
        if not client.wait_for_server(timeout_sec=0.5):
            logger.warning("ArmRosBridge: MoveJoints action server not available.")
            self.move_completed.emit(False, -1.0)
            return

        goal = MoveJoints.Goal()
        goal.joints = [float(j) for j in joints]
        goal.duration = duration

        self.move_started.emit('joints')
        future = client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        future.add_done_callback(self._on_goal_accepted)

    def cancel_move(self):
        """Cancel the currently active action goal, if any."""
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.move_completed.emit(False, -1.0)
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.move_feedback.emit(float(fb.current_error), float(fb.elapsed_time))

    def _on_action_result(self, future):
        result = future.result().result
        self._active_goal_handle = None
        self.move_completed.emit(bool(result.success), float(result.final_error))

    # --- Callback registration (non-Qt, for services layer) ---

    def on_feedback(self, callback: Callable[[dict], None]):
        """Register a callback for motor feedback (dict with positions, etc.)."""
        if self._node is not None:
            self._node._feedback_callbacks.append(callback)

    # --- Lifecycle ---

    def shutdown(self):
        """Clean shutdown of ROS 2 resources."""
        if self._spin_timer:
            self._spin_timer.stop()
        if self._node:
            self._node.destroy_node()
            self._node = None
        try:
            if _ROS_AVAILABLE and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self._ros_available = False
