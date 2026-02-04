import rclpy
from rclpy.node import Node
import numpy as np
import casadi as ca
import math
import os

from poker_interfaces.msg import TargetPose, ServoCommand, MotorFeedback, TargetJoints
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

class PokerController(Node):
    def __init__(self):
        super().__init__('poker_controller')
        
        # --- Robot Configuration ---
        self.n_joints = 6
        self.SERVO_ZERO = [820, 850, 467, 1851, 1069, 3050]
        self.SERVO_SIGNS = [1, 1, 1, 1, 1, 1] 
        self.STEPS_PER_RAD = 4096.0 / (2.0 * np.pi)
        
        # Velocity Limits
        self.MAX_RPM = 50.0 
        self.Q_MAX_STEPS = (self.MAX_RPM * 4096.0) / 60.0 
        self.Q_MIN_STEPS = 50.0 
        
        # --- Matrix LQR Design ---
        # Ts matches the hardware loop rate (10ms = 100Hz)
        self.Ts = 0.01
        
        # Q and R matrices (Diagonal initialization)
        q_diag = [100.0] * self.n_joints
        r_diag = [1.0] * self.n_joints
        
        self.Q_mat = np.diag(q_diag)
        self.R_mat = np.diag(r_diag)
        
        # --- Analytical DARE Solution ---
        # P = 0.5 * Q^0.5 * [I + (I + 4/Ts^2 * Q^-0.5 * R * Q^-0.5)^0.5] * Q^0.5
        
        # Calculate Q^0.5 (Element-wise sqrt of diagonal)
        q_diag_sqrt = np.sqrt(q_diag)
        Q_sqrt = np.diag(q_diag_sqrt)

        # Calculate Q^-0.5 (Element-wise reciprocal of diagonal)
        Q_inv_sqrt = np.diag(1.0 / q_diag_sqrt)
        I = np.eye(self.n_joints)
        
        # Inner term: R_term = Q^-0.5 * R * Q^-0.5
        R_term = Q_inv_sqrt @ self.R_mat @ Q_inv_sqrt
        
        # Bracket term: (I + 4/Ts^2 * R_term)^0.5
        inner_root = np.sqrt(I + (4.0 / self.Ts**2) * R_term)
        
        # Calculate P
        self.P_mat = 0.5 * Q_sqrt @ (I + inner_root) @ Q_sqrt
        
        # Calculate LQR Gain K (K = Ts * (R + Ts^2*P)^-1 * P)
        inv_term = np.linalg.inv(self.R_mat + (self.Ts**2) * self.P_mat)
        self.K_mat = self.Ts * inv_term @ self.P_mat
        
        self.get_logger().info(f"LQR Gain K Computed:\n{self.K_mat}")

        # --- Load Kinematics ---
        self.ik_func = None
        try:
            pkg_dir = get_package_share_directory('poker_control')
            model_dir = os.path.join(pkg_dir, 'models')
            self.ik_func = ca.Function.load(os.path.join(model_dir, 'inverse_kinematics.casadi'))
        except Exception as e:
            self.get_logger().error(f"Failed to load Kinematics: {e}")

        # --- State Management ---
        self.q_current_internal = np.zeros(self.n_joints)
        self.q_measured = np.zeros(self.n_joints)
        self.has_feedback = False
        
        self.q_ref_traj = [] 
        self.v_ref_traj = []
        self.traj_index = 0
        self.tracking_active = False
        self.goal_reached_logged = False

        # --- Interfaces ---
        self.sub_pose = self.create_subscription(
            TargetPose, '/target_pose', self.pose_callback, 10)
            
        self.sub_joints = self.create_subscription(
            TargetJoints, '/target_joints', self.joints_callback, 10)
        
        self.sub_feedback = self.create_subscription(
            MotorFeedback, '/motor_feedback', self.feedback_callback, 10)
        
        self.pub_cmd = self.create_publisher(
            ServoCommand, '/servo_cmd', 10)
        
        self.pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)
        
        self.pub_ik_solution = self.create_publisher(
            Float64MultiArray, '/ik_solution', 10)
        
        self.joint_names = ['1', '2', '3', '4', '5', '6']

    def feedback_callback(self, msg):
        # 1. Process Measurement
        if len(msg.positions) < self.n_joints: return
        for i in range(self.n_joints):
            steps = msg.positions[i]
            if steps == -1: continue
            diff = steps - self.SERVO_ZERO[i]
            self.q_measured[i] = diff / (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
        
        self.has_feedback = True

        # 2. Trigger Control Update Immediately
        self.update_control()

    def joints_callback(self, msg):
        if len(msg.joints) != self.n_joints:
            self.get_logger().error("Received invalid joint command")
            return

        q_target = np.array(msg.joints)
        self.get_logger().info(f"Moving to Joints: {q_target} in {msg.duration}s")
        self.plan_move(q_target, duration=msg.duration)

    def pose_callback(self, msg):
        if self.ik_func is None:
            self.get_logger().error("IK Function not loaded.")
            return

        self.get_logger().info(f"Planning move to: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")
        try:
            target_vec = [msg.x, msg.y, msg.z, msg.pitch, msg.roll]
            res = self.ik_func(target_vec)
            q_target = np.array(res).flatten()
            
            ik_msg = Float64MultiArray()
            ik_msg.data = q_target.tolist()
            self.pub_ik_solution.publish(ik_msg)
            
            self.get_logger().info(f"IK Solved: {q_target}")
            self.plan_move(q_target, msg.duration)
        except Exception as e:
            self.get_logger().error(f"Planning Failed: {e}")

    def plan_move(self, q_target, duration):
        self.get_logger().info(f"Moving to target in {duration}s...")

        self.start_time = self.get_clock().now()

        q_start = self.q_measured if self.has_feedback else self.q_current_internal
        self.generate_trajectory(q_start, q_target, duration)

        self.traj_index = 0
        self.tracking_active = True
        self.goal_reached_logged = False
        
        if not self.has_feedback:
            self.update_control()

    def generate_trajectory(self, q_start, q_end, duration):
        steps = int(duration / self.Ts)
        if steps <= 0: steps = 1
        t_vec = np.linspace(0, duration, steps)
        
        self.q_ref_traj = [np.zeros(self.n_joints) for _ in range(steps)]
        self.v_ref_traj = [np.zeros(self.n_joints) for _ in range(steps)]
        
        for j in range(self.n_joints):
            s, e = q_start[j], q_end[j]
            a0 = s
            a3 = 10 * (e - s) / (duration**3)
            a4 = -15 * (e - s) / (duration**4)
            a5 = 6 * (e - s) / (duration**5)
            
            q_j = a0 + a3*(t_vec**3) + a4*(t_vec**4) + a5*(t_vec**5)
            v_j = 3*a3*(t_vec**2) + 4*a4*(t_vec**3) + 5*a5*(t_vec**4)
            
            for k in range(steps):
                self.q_ref_traj[k][j] = q_j[k]
                self.v_ref_traj[k][j] = v_j[k]

    def rad_to_servo(self, q_rad_array):
        cmd_steps = []
        for i in range(self.n_joints):
            steps = self.SERVO_ZERO[i] + (q_rad_array[i] * self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            steps = max(0, min(4095, int(steps)))
            cmd_steps.append(steps)
        return cmd_steps

    def update_control(self):
        """Called synchronously when feedback arrives"""
        # FIX: Do not return if tracking is inactive. 
        # Instead, just hold the current position if we aren't moving.
        if not self.tracking_active:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        # Determine the current index based on elapsed time
        self.traj_index = int(elapsed / self.Ts)

        # Check if trajectory is finished
        if self.traj_index >= len(self.q_ref_traj):
            # Hold the last point
            self.traj_index = len(self.q_ref_traj) - 1
            
            # --- NEW: Log Success Once ---
            if not self.goal_reached_logged:
                self.get_logger().info(f"✅ GOAL REACHED (Actual time: {elapsed:.2f}s)")
                self.goal_reached_logged = True
            
        q_des = self.q_ref_traj[self.traj_index]
        v_des = self.v_ref_traj[self.traj_index]
        
        # Next Desired State (k+1) logic handles the "hold" automatically
        # because if traj_index is at the end, q_des_next == q_des
        if self.traj_index + 1 < len(self.q_ref_traj):
            q_des_next = self.q_ref_traj[self.traj_index + 1]
        else:
            q_des_next = q_des

        # 1. Calculate Error
        if self.has_feedback:
            e = self.q_measured - q_des
        else:
            e = np.zeros(6) 

        # 2. Calculate Control Effort (Matrix Version)
        K_e = self.K_mat @ e
        u_tilde = v_des - K_e
        
        # 3. Calculate Speed Limits
        cmd_speed = []
        for j in range(self.n_joints):
            u_steps_s = abs(u_tilde[j] * self.STEPS_PER_RAD)
            limit = min(self.Q_MAX_STEPS, max(self.Q_MIN_STEPS, 1.5 * u_steps_s))
            cmd_speed.append(int(limit))

        # 4. Calculate Position Command
        correction = e - (self.Ts * K_e)
        q_cmd_rad = q_des_next + correction
        cmd_pos = self.rad_to_servo(q_cmd_rad)
        cmd_acc = [100] * 6

        msg = ServoCommand()
        msg.ids = [1, 2, 3, 4, 5, 6]
        msg.position = cmd_pos
        msg.speed = cmd_speed
        msg.acceleration = cmd_acc
        self.pub_cmd.publish(msg)

        # Publish JointStates
        current_rads = self.q_measured if self.has_feedback else self.q_current_internal
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = current_rads.tolist()
        self.pub_joint_states.publish(js_msg)

        # Only increment if we haven't reached the end
        if self.traj_index < len(self.q_ref_traj):
            self.q_current_internal = q_des # Update internal state for open loop

def main(args=None):
    rclpy.init(args=args)
    node = PokerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()