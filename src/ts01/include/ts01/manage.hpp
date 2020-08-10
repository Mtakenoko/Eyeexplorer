#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "TS_01.hpp"

#define LOOP_RATE 1000 // [Hz]
#define DIGITAL_INPUT 10
#define ANALOG_INPUT 16
#define COUNT_INPUT 3

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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_stage_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_pullout_;
    void topic_callback_stage(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void topic_callback_pullout(const std_msgs::msg::Bool::SharedPtr msg_pullout_);
    // Publish
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_status_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_encoder_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_di_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ai_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_count_;
    // publish用msg
    std_msgs::msg::Bool::SharedPtr msg_status_;
    sensor_msgs::msg::JointState::SharedPtr msg_encoder_;
    std_msgs::msg::Int32MultiArray::SharedPtr msg_di_;
    std_msgs::msg::Float32MultiArray::SharedPtr msg_ai_;
    std_msgs::msg::Int32MultiArray::SharedPtr msg_count_;
    
    Manage_EyeExplorer eyeexplorer;
};