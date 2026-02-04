import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from poker_interfaces.msg import ServoCommand, MotorFeedback

class SimBridge(Node):
    def __init__(self):
        super().__init__('sim_bridge')
        
        # --- Config ---
        self.n_joints = 6
        self.joint_names = ['1', '2', '3', '4', '5', '6'] 
        self.SERVO_ZERO = [820, 850, 467, 1851, 1069, 3050]
        self.SERVO_SIGNS = [1, 1, 1, 1, 1, 1] 
        self.STEPS_PER_RAD = 4096.0 / (2.0 * np.pi)

        # --- Publishers/Subscribers ---
        self.sub_cmd = self.create_subscription(
            ServoCommand, '/servo_cmd', self.cmd_callback, 10)
            
        self.pub_fwd = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)

        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
            
        self.pub_feedback = self.create_publisher(
            MotorFeedback, '/motor_feedback', 10)

    def cmd_callback(self, msg):
        if len(msg.position) != self.n_joints: return
        rads = []
        for i in range(self.n_joints):
            steps = msg.position[i]
            diff = steps - self.SERVO_ZERO[i]
            r = diff / (self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            rads.append(r)

        # UPDATED: Send simple Float64MultiArray
        cmd_msg = Float64MultiArray()
        cmd_msg.data = rads
        self.pub_fwd.publish(cmd_msg)

    def joint_state_cb(self, msg):
        if len(msg.name) < self.n_joints: return
        
        sim_positions = [0.0]*self.n_joints
        sim_velocities = [0.0]*self.n_joints
        
        name_map = {name: i for i, name in enumerate(msg.name)}
        
        try:
            for i, target_name in enumerate(self.joint_names):
                idx = name_map[target_name]
                sim_positions[i] = msg.position[idx]
                
                # Safe access to velocity
                if len(msg.velocity) > idx:
                    sim_velocities[i] = msg.velocity[idx]
                else:
                    sim_velocities[i] = 0.0
                    
        except KeyError:
            return 

        fb_msg = MotorFeedback()
        fb_msg.feedback_valid = True
        
        for i in range(self.n_joints):
            # Position
            steps = self.SERVO_ZERO[i] + (sim_positions[i] * self.STEPS_PER_RAD * self.SERVO_SIGNS[i])
            fb_msg.positions.append(int(steps))
            
            # Speed
            vel = sim_velocities[i]
            if np.isnan(vel):
                vel = 0.0
                
            steps_vel = vel * self.STEPS_PER_RAD * self.SERVO_SIGNS[i]
            fb_msg.speeds.append(int(steps_vel))
            
            fb_msg.servo_ids.append(i+1)
            fb_msg.loads.append(0)
            fb_msg.voltages.append(0)
            fb_msg.temperatures.append(0)
            
        self.pub_feedback.publish(fb_msg)

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