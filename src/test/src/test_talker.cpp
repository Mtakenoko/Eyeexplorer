#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "test_talker.h"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher()
: Node("minimal_publisher"), count_(0){
  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",qos);

  timer_ = this->create_wall_timer(
    500ms,
    [this](){
      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "Hello " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Pub:%s", msg->data.c_str());
      publisher_->publish(*msg);
    }
  );
}