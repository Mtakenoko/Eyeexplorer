#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "TS_01.hpp"

#define LOOP_RATE 1000 // [Hz]

class Manager : public rclcpp::Node
{
public:
    Manager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    Manager(const std::string &name_space,
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void initialize();
    void readData();
    void setMessage();
    void publish();
    void detatch();

private:
    // Subscribe
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    // Publish
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_status_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_encoder_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_di_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ai_;
    // msg
    std_msgs::msg::Bool::SharedPtr msg_status_;
    sensor_msgs::msg::JointState::SharedPtr msg_encoder_;
    std_msgs::msg::Int32MultiArray::SharedPtr msg_di_;
    std_msgs::msg::Float32MultiArray::SharedPtr msg_ai_;
    
    Manage_EyeExplorer eyeexplorer;
};