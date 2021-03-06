#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

std_msgs::msg::Int32MultiArray AI_msg;

void DI_information(const std_msgs::msg::Int32MultiArray::SharedPtr sub_msg, rclcpp::Logger logger)
{
    float DI_port[10];
    static int count = 0;
    count++;
    for (int i = 0; i < 10; i++)
    {
        DI_port[i] = sub_msg->data[i];
        if (count % 10 == 0)
            RCLCPP_INFO(logger, "DI_port[%d] = %0.1f\n", i, DI_port[i]);
    }
}

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("/ts01/digital/input");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("ts01_DI_listener");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    auto callback = [&node](const std_msgs::msg::Int32MultiArray::SharedPtr msg_sub) {
        DI_information(msg_sub, node->get_logger());
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<std_msgs::msg::Int32MultiArray>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
