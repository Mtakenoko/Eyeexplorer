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

#include "geometry_msgs/msg/transform.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/buffer_core.h>

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("tf2togeomsg");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(5ms);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    tf2_ros::Buffer buffer_(clock);
    tf2::TimePoint timepoint;
    const std::string source_frame = "endoscope";
    const std::string target_frame = "world";

    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    buffer_.canTransform(target_frame, source_frame, tf2::TimePoint(), tf2::durationFromSec(1.0));

    //Publisher
    std::string topic_pub_tip("endoscope_pose");
    auto pub_tip = node->create_publisher<geometry_msgs::msg::Transform>(topic_pub_tip, qos); // Create the image publisher with our custom QoS profile.

    while (rclcpp::ok())
    {
        try
        {
            geometry_msgs::msg::Transform tip_msg;
            geometry_msgs::msg::TransformStamped TransformStamped;
            TransformStamped = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePoint());
            tip_msg.translation.x = TransformStamped.transform.translation.x * 1000;
            tip_msg.translation.y = TransformStamped.transform.translation.y * 1000;
            tip_msg.translation.z = TransformStamped.transform.translation.z * 1000;
            tip_msg.rotation = TransformStamped.transform.rotation;
            pub_tip->publish(tip_msg);
            printf("tip_msg = [%0.2lf %0.2lf %0.2lf]\n", tip_msg.translation.x, tip_msg.translation.y, tip_msg.translation.z);
        }
        catch (tf2::TransformException ex)
        {
            RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
