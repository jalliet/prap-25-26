#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "arm_interface/msg/motor_feedback.hpp"

// 引入 SCServo 驱动头文件
#include "scservo_sdk/SCServo.h"

class SCServoFeedbackNode : public rclcpp::Node
{
public:
  SCServoFeedbackNode()
  : Node("scservo_feedback_node")
  {
    // 声明参数
    this->declare_parameter("port_name", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5}); // 默认监控5个舵机
    this->declare_parameter("feedback_rate", 10.0); // Hz，默认10Hz
    this->declare_parameter("print_feedback", true); // 是否打印到终端

    // 获取参数
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    feedback_rate_ = this->get_parameter("feedback_rate").as_double();
    print_feedback_ = this->get_parameter("print_feedback").as_bool();
    
    // 获取舵机ID列表
    std::vector<int64_t> servo_ids_int64 = this->get_parameter("servo_ids").as_integer_array();
    servo_ids_.clear();
    for (auto id : servo_ids_int64) {
      servo_ids_.push_back(static_cast<int>(id));
    }

    if (servo_ids_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No servo IDs configured!");
      return;
    }

    // 初始化串口舵机
    if (!sm_st_.begin(baud_rate_, port_name_.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo on %s", port_name_.c_str());
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "SCServo initialized successfully on %s at %d baud",
                  port_name_.c_str(), baud_rate_);
    }

    // 打印监控的舵机ID
    std::string id_list = "";
    for (size_t i = 0; i < servo_ids_.size(); i++) {
      id_list += std::to_string(servo_ids_[i]);
      if (i < servo_ids_.size() - 1) id_list += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Monitoring %zu servos, IDs: [%s]", servo_ids_.size(), id_list.c_str());
    
    // 提示是否开启打印
    if (print_feedback_) {
      RCLCPP_INFO(this->get_logger(), "Terminal printing: ENABLED");
    } else {
      RCLCPP_INFO(this->get_logger(), "Terminal printing: DISABLED (use 'ros2 topic echo' to view data)");
    }

    // 创建单个发布者（发布所有舵机的数组数据）
    feedback_publisher_ = this->create_publisher<arm_interface::msg::MotorFeedback>("motor_feedback", 10);
    RCLCPP_INFO(this->get_logger(), "Created publisher: /motor_feedback");

    // 创建定时器，按照设定频率读取反馈
    int period_ms = static_cast<int>(1000.0 / feedback_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SCServoFeedbackNode::feedback_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Feedback rate: %.1f Hz (every %d ms)", 
                feedback_rate_, period_ms);
    RCLCPP_INFO(this->get_logger(), "SCServo feedback node started!");
    
    // 打印表头
    if (print_feedback_) {
      print_header();
    }
  }

  ~SCServoFeedbackNode() {
    sm_st_.end();
    RCLCPP_INFO(this->get_logger(), "SCServo feedback node shutdown");
  }

private:
  // 打印表头
  void print_header()
  {
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "  ID | Pos  | Spd | Load | Volt(mV) | Temp(℃) | Move | Curr(mA)" << std::endl;
    std::cout << "============================================================================" << std::endl;
  }

  void feedback_timer_callback()
  {
    // 创建消息
    auto msg = arm_interface::msg::MotorFeedback();
    msg.stamp = this->now();
    
    // 预分配数组大小
    size_t num_servos = servo_ids_.size();
    msg.servo_ids.resize(num_servos);
    msg.positions.resize(num_servos);
    msg.speeds.resize(num_servos);
    msg.loads.resize(num_servos);
    msg.voltages.resize(num_servos);
    msg.temperatures.resize(num_servos);
    msg.moves.resize(num_servos);
    msg.currents.resize(num_servos);
    
    bool all_valid = true;  // 跟踪是否所有读取都成功
    
    // 遍历所有舵机，读取反馈
    for (size_t i = 0; i < num_servos; i++) {
      int servo_id = servo_ids_[i];
      msg.servo_ids[i] = servo_id;
      
      // 使用批量反馈功能（更高效）
      if (sm_st_.FeedBack(servo_id) != -1) {
        // 批量读取（-1 表示使用缓存的批量数据）
        msg.positions[i] = sm_st_.ReadPos(-1);
        msg.speeds[i] = sm_st_.ReadSpeed(-1);
        msg.loads[i] = sm_st_.ReadLoad(-1);
        msg.voltages[i] = sm_st_.ReadVoltage(-1);
        msg.temperatures[i] = sm_st_.ReadTemper(-1);
        msg.moves[i] = sm_st_.ReadMove(-1);
        msg.currents[i] = sm_st_.ReadCurrent(-1);
        
        // 打印到终端（格式化输出）
        if (print_feedback_) {
          std::cout << std::setw(4) << servo_id << " | "
                    << std::setw(4) << msg.positions[i] << " | "
                    << std::setw(3) << msg.speeds[i] << " | "
                    << std::setw(4) << msg.loads[i] << " | "
                    << std::setw(8) << msg.voltages[i] << " | "
                    << std::setw(7) << msg.temperatures[i] << " | "
                    << std::setw(4) << msg.moves[i] << " | "
                    << std::setw(8) << msg.currents[i] << std::endl;
        }
      } else {
        // 批量读取失败，尝试单独读取
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Batch feedback failed for servo %d, trying individual reads", servo_id);
        
        msg.positions[i] = sm_st_.ReadPos(servo_id);
        msg.voltages[i] = sm_st_.ReadVoltage(servo_id);
        msg.temperatures[i] = sm_st_.ReadTemper(servo_id);
        msg.speeds[i] = sm_st_.ReadSpeed(servo_id);
        msg.loads[i] = sm_st_.ReadLoad(servo_id);
        msg.currents[i] = sm_st_.ReadCurrent(servo_id);
        msg.moves[i] = sm_st_.ReadMove(servo_id);
        
        // 检查是否读取失败
        if (msg.positions[i] == -1 && msg.voltages[i] == -1 && msg.temperatures[i] == -1) {
          all_valid = false;
          RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "All feedback reads failed for servo %d", servo_id);
          
          // 打印错误标记
          if (print_feedback_) {
            std::cout << std::setw(4) << servo_id << " | "
                      << " FAILED TO READ FEEDBACK" << std::endl;
          }
        } else {
          // 打印到终端
          if (print_feedback_) {
            std::cout << std::setw(4) << servo_id << " | "
                      << std::setw(4) << msg.positions[i] << " | "
                      << std::setw(3) << msg.speeds[i] << " | "
                      << std::setw(4) << msg.loads[i] << " | "
                      << std::setw(8) << msg.voltages[i] << " | "
                      << std::setw(7) << msg.temperatures[i] << " | "
                      << std::setw(4) << msg.moves[i] << " | "
                      << std::setw(8) << msg.currents[i] << std::endl;
          }
        }
      }
    }
    
    // 打印分隔线（可选，每轮数据后）
    if (print_feedback_ && num_servos > 1) {
      std::cout << "----------------------------------------------------------------------------" << std::endl;
    }
    
    msg.feedback_valid = all_valid;
    
    // 发布消息
    feedback_publisher_->publish(msg);
  }

  // 成员变量
  SMS_STS sm_st_;
  std::string port_name_;
  int baud_rate_;
  double feedback_rate_;
  bool print_feedback_;
  std::vector<int> servo_ids_;
  
  rclcpp::Publisher<arm_interface::msg::MotorFeedback>::SharedPtr feedback_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCServoFeedbackNode>());
  rclcpp::shutdown();
  return 0;
}

/*
使用说明：

1. 默认监控5个舵机（ID 1-5）并打印：
ros2 run scservo_ros2 scservo_feedback_node

2. 自定义监控的舵机ID：
ros2 run scservo_ros2 scservo_feedback_node --ros-args \
  -p servo_ids:=[1,2,3,4,5]

3. 关闭终端打印（只发布到话题）：
ros2 run scservo_ros2 scservo_feedback_node --ros-args \
  -p print_feedback:=false

4. 调整反馈频率：
ros2 run scservo_ros2 scservo_feedback_node --ros-args \
  -p feedback_rate:=20.0

5. 查看反馈数据（现在只有一个话题）：
ros2 topic echo /motor_feedback

6. 查看反馈频率：
ros2 topic hz /motor_feedback

7. 完整示例（监控ID 1-5，20Hz，带打印）：
ros2 run scservo_ros2 scservo_feedback_node --ros-args \
  -p servo_ids:=[1,2,3,4,5] \
  -p feedback_rate:=20.0 \
  -p print_feedback:=true

终端打印格式示例：
============================================================================
  ID | Pos  | Spd | Load | Volt(mV) | Temp(℃) | Move | Curr(mA)
============================================================================
   1 | 2046 |   0 |    0 |       54 |      29 |    0 |        0
   2 | 1580 |  12 |    5 |       53 |      30 |    1 |       15
   3 | 3200 |   0 |    0 |       54 |      29 |    0 |        0
   4 | 2500 |   8 |    3 |       54 |      28 |    1 |       10
   5 | 1000 |   0 |    0 |       53 |      29 |    0 |        0
----------------------------------------------------------------------------
*/