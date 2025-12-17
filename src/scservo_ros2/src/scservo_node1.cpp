#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "scservo_ros2/msg/motor_pos.hpp"
#include <vector>

// 引入 SCServo 驱动头文件
#include "scservo_sdk/SCServo.h"

class SCServoNode : public rclcpp::Node
{
public:
  SCServoNode()
  : Node("scservo_node")
  {
    // 声明参数：串口设备和波特率
    this->declare_parameter("port_name", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("default_servo_id", 1);

    // 获取参数
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    default_servo_id_ = this->get_parameter("default_servo_id").as_int();

    // 初始化串口舵机
    if (!sm_st_.begin(baud_rate_, port_name_.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo on %s", port_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "SCServo initialized successfully on %s at %d baud", 
                  port_name_.c_str(), baud_rate_);
    }

    // 创建订阅者：使用自定义消息类型
    subscription_ = this->create_subscription<scservo_ros2::msg::MotorPos>(
      "control_cmd", 
      10, 
      std::bind(&SCServoNode::motor_pos_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "Subscribed to 'control_cmd'. Ready to receive MotorPos messages.");
  }

  // 析构函数关闭串口
  ~SCServoNode() {
    sm_st_.end();
  }

private:
  void motor_pos_callback(const scservo_ros2::msg::MotorPos::SharedPtr msg)
  {
    // 检查数组是否为空
    if (msg->position.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty position array, ignoring.");
      return;
    }

    // 判断是单舵机控制还是多舵机控制
    if (msg->servo_ids.empty()) {
      // 单舵机模式：使用 servo_id 和数组的第 [0] 个元素
      control_single_servo(msg);
    } else {
      // 多舵机模式：使用 servo_ids 数组
      control_multiple_servos(msg);
    }
  }

  // 单舵机控制
  void control_single_servo(const scservo_ros2::msg::MotorPos::SharedPtr msg)
  {
    int servo_id = (msg->servo_id > 0) ? msg->servo_id : default_servo_id_;
    int pos = msg->position[0];  // 使用第 [0] 个位置
    
    // 获取速度和加速度，如果数组为空或值为0则使用默认值
    int speed = (!msg->speed.empty() && msg->speed[0] > 0) ? msg->speed[0] : 2400;
    int acc = (!msg->acceleration.empty() && msg->acceleration[0] > 0) ? msg->acceleration[0] : 50;

    // 限制位置范围 0-4095
    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;

    // 调用驱动函数
    sm_st_.WritePosEx(servo_id, pos, speed, acc);

    RCLCPP_INFO(this->get_logger(), 
                "单舵机控制 - ID=%d, Pos=%d, Speed=%d, ACC=%d", 
                servo_id, pos, speed, acc);
  }

  // 多舵机批量控制
  void control_multiple_servos(const scservo_ros2::msg::MotorPos::SharedPtr msg)
  {
    size_t num_servos = msg->servo_ids.size();
    
    // 验证数组长度一致性
    if (msg->position.size() != num_servos) {
      RCLCPP_ERROR(this->get_logger(), 
                   "Array size mismatch: servo_ids=%zu, position=%zu", 
                   num_servos, msg->position.size());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "批量控制 %zu 个舵机", num_servos);

    // 遍历每个舵机
    for (size_t i = 0; i < num_servos; i++) {
      int servo_id = msg->servo_ids[i];
      int pos = msg->position[i];
      
      // 获取速度和加速度
      int speed = (i < msg->speed.size() && msg->speed[i] > 0) ? msg->speed[i] : 2400;
      int acc = (i < msg->acceleration.size() && msg->acceleration[i] > 0) ? msg->acceleration[i] : 50;

      // 限制位置范围
      if (pos < 0) pos = 0;
      if (pos > 4095) pos = 4095;

      // 控制舵机
      sm_st_.WritePosEx(servo_id, pos, speed, acc);

      RCLCPP_INFO(this->get_logger(), 
                  "  [%zu] ID=%d, Pos=%d, Speed=%d, ACC=%d", 
                  i, servo_id, pos, speed, acc);
    }
  }

  // 成员变量
  SMS_STS sm_st_;
  std::string port_name_;
  int baud_rate_;
  int default_servo_id_;
  rclcpp::Subscription<scservo_ros2::msg::MotorPos>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCServoNode>());
  rclcpp::shutdown();
  return 0;
}

// ros2 topic pub --once /control_cmd scservo_ros2/msg/MotorPos "{
//   servo_id: 1,
//   servo_ids: [],
//   position: [2048],
//   speed: [2400],
//   acceleration: [50]
// }"

// ros2 topic pub --once /control_cmd scservo_ros2/msg/MotorPos "{
//   servo_id: 0,
//   servo_ids: [1, 2, 3],
//   position: [2048, 1024, 3072],
//   speed: [2400, 2400, 2400],
//   acceleration: [50, 50, 50]
// }"