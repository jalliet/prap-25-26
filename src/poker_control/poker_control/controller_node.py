import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import casadi as ca
import math
import os
import time

from poker_interfaces.msg import ServoCommand, MotorFeedback
from poker_interfaces.action import MoveJoints, MovePose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

class PokerController(Node):
    def __init__(self):
        super().__init__('poker_controller')
        
        # --- Robot Configuration ---
        self.n_joints = 6
        self.SERVO_ZERO = [2048, 2048, 479, 1873, 2048, 2048]
        self.SERVO_SIGNS = [1] * self.n_joints                                                                            
        self.STEPS_PER_RAD = 4096.0 / (2.0 * np.pi)
        self.MAX_RPM = 50.0 
        self.Q_MAX_STEPS = (self.MAX_RPM * 4096.0) / 60.0 
        self.Q_MIN_STEPS = 50.0 
        
        # --- Matrix LQR Design ---
        self.Ts = 0.01
        q_diag = [5.0] * self.n_joints
        r_diag = [10.0] * self.n_joints
        self.Q_mat = np.diag(q_diag)
        self.R_mat = np.diag(r_diag)
        
        q_diag_sqrt = np.sqrt(q_diag)
        Q_sqrt = np.diag(q_diag_sqrt)
        Q_inv_sqrt = np.diag(1.0 / q_diag_sqrt)
        I = np.eye(self.n_joints)
        
        R_term = Q_inv_sqrt @ self.R_mat @ Q_inv_sqrt
        inner_root = np.sqrt(I + (4.0 / self.Ts**2) * R_term)
        self.P_mat = 0.5 * Q_sqrt @ (I + inner_root) @ Q_sqrt
        
        inv_term = np.linalg.inv(self.R_mat + (self.Ts**2) * self.P_mat)
        self.K_mat = self.Ts * inv_term @ self.P_mat

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
        
        self.current_q_target = None
        self.current_duration = 0.0
        
        # Tracking variables for Action Feedback
        self.last_error = 0.0
        self.last_elapsed = 0.0

        # --- Interfaces ---
        self.sub_feedback = self.create_subscription(
            MotorFeedback, '/motor_feedback', self.feedback_callback, 10)
        
        self.pub_cmd = self.create_publisher(
            ServoCommand, '/servo_cmd', 10)
        
        self.pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)
        
        self.pub_ik_solution = self.create_publisher(
            Float64MultiArray, '/ik_solution', 10)
        
        self.joint_names = ['1', '2', '3', '4', '5', '6']

        # --- ACTION SERVERS ---
        # Give actions their own thread pool so they don't block motor feedback!
        self.action_cb_group = MutuallyExclusiveCallbackGroup()

        self.joints_action_server = ActionServer(
            self, MoveJoints, '/move_joints',
            execute_callback=self.execute_joints_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group)

        self.pose_action_server = ActionServer(
            self, MovePose, '/move_pose',
            execute_callback=self.execute_pose_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_cb_group)

    # --- ACTION HANDLERS ---
    def goal_callback(self, goal_request):
        """Accept or reject incoming goals."""
        if self.tracking_active:
            self.get_logger().warn("Rejecting goal: Robot is currently moving!")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Allow the user to cancel a movement mid-flight."""
        self.get_logger().info("Received request to cancel movement.")
        return CancelResponse.ACCEPT

    def execute_joints_callback(self, goal_handle):
        """Handles the MoveJoints action lifecycle."""
        req = goal_handle.request
        q_target = np.array(req.joints)
        
        self.get_logger().info("Executing MoveJoints Action...")
        self.plan_move(q_target, req.duration)
        
        feedback_msg = MoveJoints.Feedback()
        
        # Loop while the controller tracks the trajectory
        while self.tracking_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.tracking_active = False # Stops the controller
                self.get_logger().info('MoveJoints Goal Canceled.')
                return MoveJoints.Result(success=False, final_error=self.last_error)
            
            # Send live feedback to the client
            feedback_msg.current_error = self.last_error
            feedback_msg.elapsed_time = self.last_elapsed
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.05)
            
        goal_handle.succeed()
        result = MoveJoints.Result(success=True, final_error=self.last_error)
        return result

    def execute_pose_callback(self, goal_handle):
        """Handles the MovePose action lifecycle, including IK."""
        req = goal_handle.request
        
        if self.ik_func is None:
            self.get_logger().error("IK Function not loaded. Aborting action.")
            goal_handle.abort()
            return MovePose.Result(success=False, final_error=99.9)

        try:
            target_vec = [req.x, req.y, req.z, req.pitch, req.roll]
            res = self.ik_func(target_vec)
            q_target = np.array(res).flatten()
            
            ik_msg = Float64MultiArray()
            ik_msg.data = q_target.tolist()
            self.pub_ik_solution.publish(ik_msg)
            
            self.get_logger().info("Executing MovePose Action (IK Solved)...")
            self.plan_move(q_target, req.duration)
            
            feedback_msg = MovePose.Feedback()
            
            while self.tracking_active:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.tracking_active = False
                    return MovePose.Result(success=False, final_error=self.last_error)
                
                feedback_msg.current_error = self.last_error
                feedback_msg.elapsed_time = self.last_elapsed
                goal_handle.publish_feedback(feedback_msg)
                
                time.sleep(0.05)
                
            goal_handle.succeed()
            return MovePose.Result(success=True, final_error=self.last_error)
            
        except Exception as e:
            self.get_logger().error(f"IK Planning Failed: {e}")
            goal_handle.abort()
            return MovePose.Result(success=False, final_error=99.9)

    # --- CORE CONTROLLER LOGIC ---
    def feedback_callback(self, msg):
        if len(msg.positions) < self.n_joints: return
        for i in range(self.n_joints):
            steps = msg.positions[i]
            if steps == -1: continue
            diff = steps - self.SERVO_ZERO[i]
            self.q_measured[i] = diff / (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
        
        self.has_feedback = True

        if self.tracking_active and self.current_q_target is not None:
            self.update_control(self.current_q_target, self.current_duration)

    def plan_move(self, q_target, duration):
        self.current_q_target = q_target
        self.current_duration = duration
        self.start_time = self.get_clock().now()

        q_start = self.q_measured if self.has_feedback else self.q_current_internal
        self.generate_trajectory(q_start, q_target, duration)

        self.traj_index = 0
        self.tracking_active = True
        self.goal_reached_logged = False
        
        if not self.has_feedback:
            self.update_control(q_target, duration)

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

    def update_control(self, q_target, duration):
        if not self.tracking_active:
            return
        
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        self.traj_index = int(elapsed / self.Ts)

        if self.traj_index >= len(self.q_ref_traj):
            self.traj_index = len(self.q_ref_traj) - 1
            
        q_des = self.q_ref_traj[self.traj_index]
        v_des = self.v_ref_traj[self.traj_index]
        
        if self.traj_index + 1 < len(self.q_ref_traj):
            q_des_next = self.q_ref_traj[self.traj_index + 1]
        else:
            q_des_next = q_des
        
        max_pos_error = np.max(np.abs(q_target - self.q_measured))
        
        # Save state for the Action Server Feedback
        self.last_error = float(max_pos_error)
        self.last_elapsed = float(elapsed)
        
        error_threshold = 0.05

        if self.traj_index == len(self.q_ref_traj) - 1:
            if not self.goal_reached_logged:
                if (max_pos_error <= error_threshold):
                    self.get_logger().info(f"✅ GOAL REACHED (Actual time: {elapsed:.2f}s, Final error: {max_pos_error:.3f} rad)")
                    self.goal_reached_logged = True
                    self.tracking_active = False
                elif elapsed >= (duration + 10):
                    self.get_logger().warn(f"Goal timeout! Final error: {max_pos_error:.3f} rad")
                    self.goal_reached_logged = True
                    self.tracking_active = False

        if self.has_feedback:
            e = self.q_measured - q_des
        else:
            e = np.zeros(6) 

        # Calculate Control Effort
        K_e = self.K_mat @ e
        
        # Calculate Speed Limits
        u_tilde = v_des - K_e 
        cmd_speed = []
        for j in range(self.n_joints):
            u_steps_s = abs(u_tilde[j] * self.STEPS_PER_RAD)
            limit = min(self.Q_MAX_STEPS, max(self.Q_MIN_STEPS, 1.5 * u_steps_s))
            cmd_speed.append(int(limit))

        # Calculate Position Command (Hybrid LQR)
        correction = e - (self.Ts * K_e)
        q_cmd_rad = q_des_next + 0.1*correction
        cmd_pos = self.rad_to_servo(q_cmd_rad)
        cmd_acc = [4000] * 6

        msg = ServoCommand()
        msg.ids = [1, 2, 3, 4, 5, 6]
        msg.position = cmd_pos
        msg.speed = cmd_speed
        msg.acceleration = cmd_acc
        self.pub_cmd.publish(msg)

        current_rads = self.q_measured if self.has_feedback else self.q_current_internal
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = self.joint_names
        js_msg.position = current_rads.tolist()
        self.pub_joint_states.publish(js_msg)

        if self.traj_index < len(self.q_ref_traj):
            self.q_current_internal = q_des

def main(args=None):
    rclpy.init(args=args)
    node = PokerController()
    
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