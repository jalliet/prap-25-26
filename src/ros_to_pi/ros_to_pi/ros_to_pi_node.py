#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray
from arm_interface.msg import MotorPos
import math

class MyDIYNode(Node):
    def __init__(self):
        super().__init__("ros_to_motor_node")

        self.get_logger().info("Waiting for joint_from_pose")

        # 创建发布者，发布到 /control_cmd 话题
        self.motor_publisher = self.create_publisher(
            MotorPos,
            '/control_cmd',
            10
        )

        # 订阅关节角度
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_from_pose',
            self.joint_callback,
            10
        )

        # 可配置的参数
        self.default_speed = 1500  # 默认速度
        self.default_acceleration = 50  # 默认加速度
        self.num_servos = 5  # 舵机数量

    def joint_callback(self, msg):
        angles = list(msg.data)
        self.get_logger().info(f"Received joint angles: {angles}")

        # 将角度转换为编码器值 (0-4095)
        encoder_vals = []
        for a in angles:
            # 将角度从 [-π, π] 归一化到 [0, 1]
            norm = (a + math.pi) / (2 * math.pi)
            # 转换为编码器值 (0-4095)
            enc = int(norm * 4095)
            # 限制在有效范围内
            enc = max(0, min(4095, enc))
            encoder_vals.append(enc)

        self.get_logger().info("Motor encoder values:")
        for i, v in enumerate(encoder_vals):
            self.get_logger().info(f"  Servo {i+1}: {v}")

        # 创建 MotorPos 消息
        motor_msg = MotorPos()
        
        # 设置舵机ID列表 (假设是1到5)
        motor_msg.servo_ids = list(range(1, len(encoder_vals) + 1))
        
        # 设置位置
        motor_msg.position = encoder_vals
        
        # 设置速度（所有舵机使用相同速度）
        motor_msg.speed = [self.default_speed] * len(encoder_vals)
        
        # 设置加速度（所有舵机使用相同加速度）
        motor_msg.acceleration = [self.default_acceleration] * len(encoder_vals)

        # 发布消息
        self.motor_publisher.publish(motor_msg)
        
        self.get_logger().info(
            f"Published motor command: IDs={motor_msg.servo_ids}, "
            f"Positions={motor_msg.position}, "
            f"Speed={motor_msg.speed[0]}, "
            f"Accel={motor_msg.acceleration[0]}\n"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MyDIYNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
