#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "../include/stage/stage.hpp"


class Controller_Stage
    : public rclcpp::Node
{
public:
    Controller_Stage(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    Controller_Stage(const std::string &name_space,
                     const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void publish();

private:
    // Subscribe
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_count_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_di_;
    void topic_callback_count(const std_msgs::msg::Int32MultiArray::SharedPtr msg_count_);
    void topic_callback_di(const std_msgs::msg::Int32MultiArray::SharedPtr msg_di_);

    Stage stage;

};