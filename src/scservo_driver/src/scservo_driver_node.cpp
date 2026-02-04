#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "poker_interfaces/msg/servo_command.hpp"
#include "poker_interfaces/msg/motor_feedback.hpp"

#include "scservo_sdk/SCServo.h"

using namespace std::chrono_literals;

class SCServoDriver : public rclcpp::Node
{
public:
  SCServoDriver() : Node("scservo_driver")
  {
    this->declare_parameter("port", "/dev/ttyACM0");
    this->declare_parameter("baud", 1000000);
    this->declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5, 6});
    
    std::string port = this->get_parameter("port").as_string();
    int baud = this->get_parameter("baud").as_int();
    auto ids = this->get_parameter("servo_ids").as_integer_array();
    for(auto id : ids) servo_ids_.push_back(static_cast<int>(id));

    if (!servo_sdk.begin(baud, port.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Port %s opened at %d", port.c_str(), baud);
    }

    sub_ = this->create_subscription<poker_interfaces::msg::ServoCommand>(
      "/servo_cmd", 10, std::bind(&SCServoDriver::cmd_cb, this, std::placeholders::_1));

    pub_feedback_ = this->create_publisher<poker_interfaces::msg::MotorFeedback>("/motor_feedback", 10);

    // INCREASED to 20Hz (50ms) - Safe limit for 6 servos @ 1Mbps
    timer_ = this->create_wall_timer(50ms, std::bind(&SCServoDriver::feedback_loop, this));
  }

  ~SCServoDriver() {
    servo_sdk.end();
  }

private:
  void cmd_cb(const poker_interfaces::msg::ServoCommand::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    size_t count = msg->ids.size();
    if(count == 0) return;

    std::vector<u8> ids(count);
    std::vector<s16> pos(count);
    std::vector<u16> spd(count);
    std::vector<u8> acc(count);

    for(size_t i=0; i<count; ++i){
        ids[i] = (u8)msg->ids[i];
        pos[i] = (s16)msg->position[i];
        spd[i] = (u16)msg->speed[i];
        acc[i] = (u8)msg->acceleration[i];
    }

    servo_sdk.SyncWritePosEx(ids.data(), count, pos.data(), spd.data(), acc.data());
  }

  void feedback_loop()
  {
    // Try to lock; if busy writing, skip this feedback cycle to prioritize control smoothness
    std::unique_lock<std::mutex> lock(serial_mutex_, std::try_to_lock);
    if(!lock.owns_lock()) {
        return; 
    }

    auto msg = poker_interfaces::msg::MotorFeedback();
    msg.stamp = this->now();
    msg.feedback_valid = true;

    for(int id : servo_ids_) {
        // Read ONLY Position and Speed to save bus time
        int pos = servo_sdk.ReadPos(id);
        int spd = servo_sdk.ReadSpeed(id);
        
        if (pos == -1) {
            msg.feedback_valid = false;
        }

        msg.servo_ids.push_back(id);
        msg.positions.push_back(pos);
        msg.speeds.push_back(spd);
        
        // Fill others with 0 to save bandwidth
        msg.loads.push_back(0);
        msg.voltages.push_back(0);
        msg.temperatures.push_back(0);
    }

    pub_feedback_->publish(msg);
  }

  SMS_STS servo_sdk;
  std::mutex serial_mutex_;
  std::vector<int> servo_ids_;
  rclcpp::Subscription<poker_interfaces::msg::ServoCommand>::SharedPtr sub_;
  rclcpp::Publisher<poker_interfaces::msg::MotorFeedback>::SharedPtr pub_feedback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCServoDriver>());
  if(rclcpp::ok()){
    rclcpp::shutdown();
  }
  return 0;
}
