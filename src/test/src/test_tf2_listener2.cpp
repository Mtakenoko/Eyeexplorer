#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/transform_listener.h"

#include <ktl.h>

using namespace std::chrono_literals;

geometry_msgs::msg::Transform tip_msg;
sensor_msgs::msg::JointState q_msg;
geometry_msgs::msg::TransformStamped tf_msg;

void forward_kinematics(const geometry_msgs::msg::TransformStamped::SharedPtr sub_msg, rclcpp::Clock::SharedPtr clock,
                        rclcpp::Logger logger, std::shared_ptr<rclcpp::Node> node)
{
    std::string honya= sub_msg->child_frame_id;
    std::string hon = sub_msg->header.frame_id;
    double trans[3];
    trans[0] = sub_msg->transform.translation.x;
    trans[1] = sub_msg->transform.translation.y;
    trans[2] = sub_msg->transform.translation.z;
    tf2_ros::TransformListener
}

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("tf");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("test_tf2_listener2");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);


    auto callback = [clock, &node](const geometry_msgs::msg::TransformStamped::SharedPtr msg_sub) {
        forward_kinematics(msg_sub, clock, node->get_logger(), node);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
