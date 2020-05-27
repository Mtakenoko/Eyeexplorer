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

#include <ktl.h>

#include "sensor_msgs/msg/joint_state.hpp"

#include "../include/arm/encoder.h"

ReadEncoder readencoder;
sensor_msgs::msg::JointState q_msg;

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("ts01_encoder");
    std::string topic_pub_q("joint_states");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("joint_publisher");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_q.c_str());
    auto pub_q = node->create_publisher<sensor_msgs::msg::JointState>(topic_pub_q, 10); // Create the image publisher with our custom QoS profile.

    //setting q_msg
    q_msg.name.push_back("arm_joint1");
    q_msg.name.push_back("arm_joint2");
    q_msg.name.push_back("arm_joint_horiz1");
    q_msg.name.push_back("arm_joint3");
    q_msg.name.push_back("arm_joint_horiz2");
    q_msg.name.push_back("arm_joint4");
    q_msg.name.push_back("arm_joint5");
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);

    //エンコーダのオフセット設定
    readencoder.SetOffset();
    // readencoder.ReadOffsetdat();

    auto callback = [pub_q, clock](const sensor_msgs::msg::JointState::SharedPtr msg_sub) {
        q_msg.position[0] = msg_sub->position[0] - readencoder.offset[0];
        q_msg.position[1] = msg_sub->position[1] - readencoder.offset[1];
        q_msg.position[2] = -q_msg.position[1];
        q_msg.position[3] = msg_sub->position[2] - readencoder.offset[2] - q_msg.position[1];
        q_msg.position[4] = -q_msg.position[3];
        q_msg.position[5] = msg_sub->position[3] - readencoder.offset[3];
        q_msg.position[6] = msg_sub->position[4] - readencoder.offset[4];
        q_msg.header.stamp = clock->now();
        pub_q->publish(q_msg);

        static int count = 0;
        if (count % 10 == 0)
        {
            printf("q = [%0.4lf %0.4lf %0.4lf %0.4lf %0.4lf %0.4lf]\n", q_msg.position[0], q_msg.position[1], q_msg.position[2], q_msg.position[4], q_msg.position[5], q_msg.position[6]);
        }
        count++;
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
