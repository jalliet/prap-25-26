#include <memory>
#include <string>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "arm_interface/msg/motor_pos.hpp"

#include "scservo_sdk/SCServo.h"

class SCServoNode : public rclcpp::Node
{
public:
  SCServoNode()
  : Node("scservo_node")
  {
    this->declare_parameter("port_name", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("default_servo_id", 1);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    default_servo_id_ = this->get_parameter("default_servo_id").as_int();

    if (!sm_st_.begin(baud_rate_, port_name_.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo on %s", port_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "SCServo initialized successfully on %s at %d baud",
                  port_name_.c_str(), baud_rate_);
    }

    subscription_ = this->create_subscription<arm_interface::msg::MotorPos>(
      "control_cmd",
      10,
      std::bind(&SCServoNode::motor_pos_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to 'control_cmd'. Ready to receive MotorPos messages.");
  }

  ~SCServoNode() {
    sm_st_.end();
  }

private:
  void motor_pos_callback(const arm_interface::msg::MotorPos::SharedPtr msg)
  {
    if (msg->position.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty position array, ignoring.");
      return;
    }

    if (msg->servo_ids.empty()) {
      control_single_servo(msg);
    } else {
      control_multiple_servos(msg);
    }
  }

  void control_single_servo(const arm_interface::msg::MotorPos::SharedPtr msg)
  {
    int servo_id = (msg->servo_id > 0) ? msg->servo_id : default_servo_id_;
    int pos = msg->position[0];

    int speed = (!msg->speed.empty() && msg->speed[0] > 0) ? msg->speed[0] : 2400;
    int acc = (!msg->acceleration.empty() && msg->acceleration[0] > 0) ? msg->acceleration[0] : 50;

    if (pos < 0) pos = 0;
    if (pos > 4095) pos = 4095;

    sm_st_.WritePosEx(servo_id, pos, speed, acc);

    RCLCPP_INFO(this->get_logger(),
                "Single servo control - ID=%d, Pos=%d, Speed=%d, ACC=%d",
                servo_id, pos, speed, acc);
  }

  void control_multiple_servos(const arm_interface::msg::MotorPos::SharedPtr msg)
  {
    size_t num_servos = msg->servo_ids.size();

    if (msg->position.size() != num_servos) {
      RCLCPP_ERROR(this->get_logger(),
                   "Array size mismatch: servo_ids=%zu, position=%zu",
                   num_servos, msg->position.size());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sync control %zu servos", num_servos);

    std::vector<u8>  ids(num_servos);
    std::vector<s16> positions(num_servos);
    std::vector<u16> speeds(num_servos);
    std::vector<u8>  accs(num_servos);

    for (size_t i = 0; i < num_servos; i++) {
      int servo_id = msg->servo_ids[i];
      int pos = msg->position[i];

      int speed = (i < msg->speed.size() && msg->speed[i] > 0) ? msg->speed[i] : 2400;
      int acc   = (i < msg->acceleration.size() && msg->acceleration[i] > 0) ? msg->acceleration[i] : 50;

      if (pos < 0) pos = 0;
      if (pos > 4095) pos = 4095;

      ids[i]       = static_cast<u8>(servo_id);
      positions[i] = static_cast<s16>(pos);
      speeds[i]    = static_cast<u16>(speed);
      accs[i]      = static_cast<u8>(acc);

      RCLCPP_INFO(this->get_logger(),
                  "  [%zu] ID=%d, Pos=%d, Speed=%d, ACC=%d",
                  i, servo_id, pos, speed, acc);
    }

    sm_st_.SyncWritePosEx(
      ids.data(),
      static_cast<u8>(ids.size()),
      positions.data(),
      speeds.data(),
      accs.data());
  }

  SMS_STS sm_st_;
  std::string port_name_;
  int baud_rate_;
  int default_servo_id_;
  rclcpp::Subscription<arm_interface::msg::MotorPos>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCServoNode>());
  rclcpp::shutdown();
  return 0;
}
