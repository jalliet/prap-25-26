"""
Mock ROS 2 arm controller for development testing.

Usage:
    python scripts/mock_arm_server.py

Provides:
  - Action server /move_pose  (MovePose)
  - Action server /move_joints (MoveJoints)
  - Publisher /motor_feedback  (MotorFeedback) at 10 Hz
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from poker_interfaces.msg import MotorFeedback
from poker_interfaces.action import MovePose, MoveJoints


class MockArmServer(Node):
    def __init__(self):
        super().__init__('mock_arm_server')

        self.action_cb_group = MutuallyExclusiveCallbackGroup()

        self.pose_server = ActionServer(
            self, MovePose, '/move_pose',
            execute_callback=self._execute_pose,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.action_cb_group)

        self.joints_server = ActionServer(
            self, MoveJoints, '/move_joints',
            execute_callback=self._execute_joints,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.action_cb_group)

        self.pub_feedback = self.create_publisher(MotorFeedback, '/motor_feedback', 10)
        self.create_timer(0.1, self._publish_feedback)  # 10 Hz

        self.get_logger().info('Mock Arm Server ready.')

    def _publish_feedback(self):
        msg = MotorFeedback()
        msg.stamp = self.get_clock().now().to_msg()
        msg.servo_ids = [1, 2, 3, 4, 5, 6]
        msg.positions = [2048] * 6  # centered
        msg.speeds = [0] * 6
        msg.loads = [0] * 6
        msg.voltages = [120] * 6
        msg.temperatures = [25] * 6
        msg.feedback_valid = True
        self.pub_feedback.publish(msg)

    def _execute_pose(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(
            f'Mock MovePose: ({req.x:.2f}, {req.y:.2f}, {req.z:.2f}) '
            f'dur={req.duration:.1f}s')
        return self._simulate_motion(goal_handle, MovePose, req.duration)

    def _execute_joints(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(
            f'Mock MoveJoints: {[f"{j:.2f}" for j in req.joints]} '
            f'dur={req.duration:.1f}s')
        return self._simulate_motion(goal_handle, MoveJoints, req.duration)

    def _simulate_motion(self, goal_handle, action_type, duration):
        feedback_msg = action_type.Feedback()
        steps = max(1, int(duration / 0.05))
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = action_type.Result()
                result.success = False
                result.final_error = 0.5
                return result
            progress = (i + 1) / steps
            feedback_msg.current_error = max(0.0, 1.0 - progress)
            feedback_msg.elapsed_time = (i + 1) * 0.05
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.05)

        goal_handle.succeed()
        result = action_type.Result()
        result.success = True
        result.final_error = 0.001
        return result


def main():
    rclpy.init()
    node = MockArmServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
