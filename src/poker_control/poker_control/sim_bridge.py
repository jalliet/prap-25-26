#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from poker_interfaces.msg import ServoCommand, MotorFeedback, TargetPose


class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')

        # --- Parameters ---
        # Allow launch file to define if we are in pure sim or digital twin
        self.declare_parameter('mode', 'digital_twin')
        self.mode = self.get_parameter('mode').value

        # --- Config ---
        self.n_joints = 6
        self.joint_names = ['1', '2', '3', '4', '5', '6']
        self.SERVO_ZERO = [1848, 1906, 500, 1859, 2050, 2013]
        self.SERVO_SIGNS = [1] * self.n_joints
        self.STEPS_PER_RAD = 4096.0 / (2.0 * np.pi)

        self.latest_real_positions_rad = []

        # --- Shared Publishers/Subscribers ---
        # Forward actual servo commands to Gazebo in both modes so it moves!
        self.sub_cmd = self.create_subscription(
            ServoCommand, '/servo_cmd', self.cmd_callback, 10)

        self.pub_fwd = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)

        if self.mode == 'sim':
            self.get_logger().info("SimBridge initialized in PURE SIM mode.")
            # Read from Gazebo, publish fake hardware feedback
            self.sub_joint_states = self.create_subscription(
                JointState, '/joint_states', self.joint_state_cb, 10)
            self.pub_feedback = self.create_publisher(
                MotorFeedback, '/motor_feedback', 10)

        else:  # digital_twin mode
            self.get_logger().info("SimBridge initialized in DIGITAL TWIN mode.")
            # Read from real hardware, snap Gazebo when new target arrives
            self.sub_real_feedback = self.create_subscription(
                MotorFeedback, '/motor_feedback', self.real_feedback_cb, 10)
            self.sub_target = self.create_subscription(
                TargetPose, '/target_pose', self.target_cb, 10)

    def cmd_callback(self, msg):
        """Forwards standard controller step commands to Gazebo in radians."""
        if len(msg.position) != self.n_joints:
            return
        rads = []
        for i in range(self.n_joints):
            steps = msg.position[i]
            diff = steps - self.SERVO_ZERO[i]
            r = diff / (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            rads.append(r)

        cmd_msg = Float64MultiArray()
        cmd_msg.data = rads
        self.pub_fwd.publish(cmd_msg)

    # ==========================================
    # PURE SIMULATION MODE FUNCTIONS
    # ==========================================
    def joint_state_cb(self, msg):
        """Takes Gazebo states and publishes them as if they came from real motors."""
        if len(msg.name) < self.n_joints:
            return

        sim_positions = [0.0]*self.n_joints
        sim_velocities = [0.0]*self.n_joints
        name_map = {name: i for i, name in enumerate(msg.name)}

        try:
            for i, target_name in enumerate(self.joint_names):
                idx = name_map[target_name]
                sim_positions[i] = msg.position[idx]
                if len(msg.velocity) > idx:
                    sim_velocities[i] = msg.velocity[idx]
                else:
                    sim_velocities[i] = 0.0
        except KeyError:
            return

        fb_msg = MotorFeedback()
        fb_msg.feedback_valid = True

        for i in range(self.n_joints):
            steps = self.SERVO_ZERO[i] + (sim_positions[i]
                                          * self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            fb_msg.positions.append(int(steps))

            vel = sim_velocities[i] if not np.isnan(sim_velocities[i]) else 0.0
            steps_vel = vel * self.STEPS_PER_RAD * self.SERVO_SIGNS[i]
            fb_msg.speeds.append(int(steps_vel))

            fb_msg.servo_ids.append(i+1)
            fb_msg.loads.append(0)
            fb_msg.voltages.append(0)
            fb_msg.temperatures.append(0)

        self.pub_feedback.publish(fb_msg)

    # ==========================================
    # DIGITAL TWIN MODE FUNCTIONS
    # ==========================================
    def real_feedback_cb(self, msg):
        """Records the real robot's position to prepare for snapping Gazebo."""
        if len(msg.positions) != self.n_joints:
            return
        rads = []
        for i in range(self.n_joints):
            diff = msg.positions[i] - self.SERVO_ZERO[i]
            r = diff / (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            rads.append(r)
        self.latest_real_positions_rad = rads

    def target_cb(self, msg):
        """When a new target is requested, instantly jump Gazebo to the real robot's current pose."""
        if not self.latest_real_positions_rad:
            return

        self.get_logger().info("Syncing Gazebo Digital Twin to real hardware position...")
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.latest_real_positions_rad
        self.pub_fwd.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
