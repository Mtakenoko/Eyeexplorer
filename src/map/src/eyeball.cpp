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

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
visualization_msgs::msg::Marker marker_msg;

void eye_ball(const sensor_msgs::msg::JointState::SharedPtr msg_sub, 
                std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> pub_marker, 
                rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
{
    //set marker
    std::string ns = "test_ns";
    int id = 0;
    marker_msg.header.frame_id = "world";
    marker_msg.header.stamp = clock->now();
    marker_msg.ns = ns;
    marker_msg.id = ++id;

    marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;

    marker_msg.scale.x = 0.24;
    marker_msg.scale.y = 1.0;
    marker_msg.scale.z = 0.5;

    marker_msg.color.a = 1.0;
    marker_msg.color.r = 0.0;
    marker_msg.color.g = 1.0;
    marker_msg.color.b = 0.0;

    marker_msg.pose.position.x = 0;
    marker_msg.pose.position.y = 0;
    marker_msg.pose.position.z = -2;

    //Publish
    RCLCPP_INFO(logger, "Publishing Sphere");
    pub_marker->publish(marker_msg);
}

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub_cloud("joint_states");
    std::string topic_pub_marker("est_eyeball");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("eye_ball");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_marker.c_str());
    auto pub_marker = node->create_publisher<visualization_msgs::msg::Marker>(topic_pub_marker, 10); // Create the image publisher with our custom QoS profile.

    auto callback = [pub_marker, clock, &node](const sensor_msgs::msg::JointState::SharedPtr msg_sub) {
        eye_ball(msg_sub, pub_marker, clock, node->get_logger());
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub_cloud.c_str());
    auto sub_pointcloud = node->create_subscription<sensor_msgs::msg::JointState>(topic_sub_cloud, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
