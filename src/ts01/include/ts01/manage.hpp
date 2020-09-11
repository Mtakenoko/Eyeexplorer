#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "TS_01.hpp"

#define LOOP_RATE 1000 // [Hz]
#define COUNT_INPUT 3
#define TOPIC_TS01_STATUS "/ts01/status"
#define TOPIC_TS01_ENCODER "/ts01/encoder"
#define TOPIC_TS01_DIN "/ts01/din"
#define TOPIC_TS01_DOUT "/ts01/dout"
#define TOPIC_TS01_AIN "/ts01/ain"
#define TOPIC_TS01_AOUT "/ts01/aout"
#define TOPIC_TS01_COUNTER "/ts01/counter"

class Manager : public rclcpp::Node
{
public:
    Manager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    Manager(const std::string &name_space,
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void initialize();
    void readData();
    void setMessage();
    void setOutput();
    void publish();
    void detatch();
    void outputData();

private:
    // Subscribe
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_stage_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_dout_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_aout_;
    void topic_callback_stage(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void topic_callback_dout(const std_msgs::msg::Bool::SharedPtr msg_dout_);
    void topic_callback_aout(const std_msgs::msg::Float32MultiArray::SharedPtr msg_aout_);

    // Publish
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_status_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_encoder_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_di_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ai_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_count_;

    // publish用msg
    std_msgs::msg::Bool msg_status;
    sensor_msgs::msg::JointState msg_encoder;
    std_msgs::msg::Int32MultiArray msg_di;
    std_msgs::msg::Float32MultiArray msg_ai;
    std_msgs::msg::Int32MultiArray msg_count;

    // Eyeexplorer
    Manage_EyeExplorer eyeexplorer;

    // Subscribeデータ保持用配列
    bool dout[TS01_DO_CH_NUM];
    bool aout[TS01_DO_CH_NUM];
};