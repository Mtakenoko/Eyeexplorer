#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "minimal_subscriber.hpp"

void MinimalSubscriber::topic_callback_(const std_msgs::msg::String::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
}

MinimalSubscriber::MinimalSubscriber()
: Node("minimal_subscriber_test"){
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "chatter",
    std::bind(&MinimalSubscriber::topic_callback_, this, std::placeholders::_1)
  );
}