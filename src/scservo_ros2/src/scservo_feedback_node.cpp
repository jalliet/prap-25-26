#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "arm_interface/msg/motor_feedback.hpp"

#include "scservo_sdk/SCServo.h"

class SCServoFeedbackNode : public rclcpp::Node
{
public:
  SCServoFeedbackNode()
  : Node("scservo_feedback_node")
  {
    this->declare_parameter("port_name", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 1000000);
    this->declare_parameter("servo_ids", std::vector<int64_t>{1, 2, 3, 4, 5});
    this->declare_parameter("feedback_rate", 10.0);
    this->declare_parameter("print_feedback", true);

    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    feedback_rate_ = this->get_parameter("feedback_rate").as_double();
    print_feedback_ = this->get_parameter("print_feedback").as_bool();
    
    std::vector<int64_t> servo_ids_int64 = this->get_parameter("servo_ids").as_integer_array();
    servo_ids_.clear();
    for (auto id : servo_ids_int64) {
      servo_ids_.push_back(static_cast<int>(id));
    }

    if (servo_ids_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No servo IDs configured!");
      return;
    }

    if (!sm_st_.begin(baud_rate_, port_name_.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init SCServo on %s", port_name_.c_str());
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "SCServo initialized successfully on %s at %d baud",
                  port_name_.c_str(), baud_rate_);
    }

    std::string id_list = "";
    for (size_t i = 0; i < servo_ids_.size(); i++) {
      id_list += std::to_string(servo_ids_[i]);
      if (i < servo_ids_.size() - 1) id_list += ", ";
    }
    RCLCPP_INFO(this->get_logger(), "Monitoring %zu servos, IDs: [%s]", servo_ids_.size(), id_list.c_str());
    
    if (print_feedback_) {
      RCLCPP_INFO(this->get_logger(), "Terminal printing: ENABLED");
    } else {
      RCLCPP_INFO(this->get_logger(), "Terminal printing: DISABLED (use 'ros2 topic echo' to view data)");
    }

    feedback_publisher_ =
      this->create_publisher<arm_interface::msg::MotorFeedback>("motor_feedback", 10);
    RCLCPP_INFO(this->get_logger(), "Created publisher: /motor_feedback");

    int period_ms = static_cast<int>(1000.0 / feedback_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SCServoFeedbackNode::feedback_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Feedback rate: %.1f Hz (every %d ms)", 
                feedback_rate_, period_ms);
    RCLCPP_INFO(this->get_logger(), "SCServo feedback node started!");
    
    if (print_feedback_) {
      print_header();
    }
  }

  ~SCServoFeedbackNode() {
    sm_st_.end();
    RCLCPP_INFO(this->get_logger(), "SCServo feedback node shutdown");
  }

private:
  void print_header()
  {
    std::cout << "\n============================================================================" << std::endl;
    std::cout << "  ID | Pos  | Spd | Load | Volt(mV) | Temp(℃) | Move | Curr(mA)" << std::endl;
    std::cout << "============================================================================" << std::endl;
  }

  void feedback_timer_callback()
  {
    auto msg = arm_interface::msg::MotorFeedback();
    msg.stamp = this->now();
    
    size_t num_servos = servo_ids_.size();
    msg.servo_ids.resize(num_servos);
    msg.positions.resize(num_servos);
    msg.speeds.resize(num_servos);
    msg.loads.resize(num_servos);
    msg.voltages.resize(num_servos);
    msg.temperatures.resize(num_servos);
    msg.moves.resize(num_servos);
    msg.currents.resize(num_servos);
    
    bool all_valid = true;
    
    for (size_t i = 0; i < num_servos; i++) {
      int servo_id = servo_ids_[i];
      msg.servo_ids[i] = servo_id;
      
      if (sm_st_.FeedBack(servo_id) != -1) {
        msg.positions[i]    = sm_st_.ReadPos(-1);
        msg.speeds[i]       = sm_st_.ReadSpeed(-1);
        msg.loads[i]        = sm_st_.ReadLoad(-1);
        msg.voltages[i]     = sm_st_.ReadVoltage(-1);
        msg.temperatures[i] = sm_st_.ReadTemper(-1);
        msg.moves[i]        = sm_st_.ReadMove(-1);
        msg.currents[i]     = sm_st_.ReadCurrent(-1);
        
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Batch feedback failed for servo %d, trying individual reads", servo_id);
        
        msg.positions[i]    = sm_st_.ReadPos(servo_id);
        msg.voltages[i]     = sm_st_.ReadVoltage(servo_id);
        msg.temperatures[i] = sm_st_.ReadTemper(servo_id);
        msg.speeds[i]       = sm_st_.ReadSpeed(servo_id);
        msg.loads[i]        = sm_s_
