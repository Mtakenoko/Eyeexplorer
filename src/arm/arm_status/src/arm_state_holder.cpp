#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/buffer_core.h>

using namespace std::chrono_literals;

geometry_msgs::msg::Transform tip_msg;

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("arm_state_holder");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(1ms);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    tf2_ros::Buffer buffer_(clock);
    const std::string target_frame = "world";
    const std::string source_frame = "endoscope";

    tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    buffer_.canTransform(target_frame, source_frame, tf2::TimePoint(), tf2::durationFromSec(1.0));

    std::string topic_pub_tip("endoscope_transform");
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_tip.c_str());
    auto pub_tip = node->create_publisher<geometry_msgs::msg::Transform>(topic_pub_tip, qos); // Create the image publisher with our custom QoS profile.

    const int end_timing = 225; //[ms]だけ送らせて配信する
    double x[end_timing], y[end_timing], z[end_timing];
    double qx[end_timing], qy[end_timing], qz[end_timing], qw[end_timing];
    bool PUB_START = false;
    int timing = 0;

    while (rclcpp::ok())
    {
        try
        {
            geometry_msgs::msg::TransformStamped TransformStamped;
            TransformStamped = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePoint());

            x[timing] = TransformStamped.transform.translation.x;
            y[timing] = TransformStamped.transform.translation.y;
            z[timing] = TransformStamped.transform.translation.z;
            qx[timing] = TransformStamped.transform.rotation.x;
            qy[timing] = TransformStamped.transform.rotation.y;
            qz[timing] = TransformStamped.transform.rotation.z;
            qw[timing] = TransformStamped.transform.rotation.w;
            // printf("position_now   = [%0.4f %0.4f %0.4f]\n", x[timing], y[timing], z[timing]);

            if (PUB_START)
            {
                if (timing == end_timing - 1)
                {
                    tip_msg.translation.x = x[0];
                    tip_msg.translation.y = y[0];
                    tip_msg.translation.z = z[0];
                    tip_msg.rotation.x = qx[0];
                    tip_msg.rotation.y = qy[0];
                    tip_msg.rotation.z = qz[0];
                    tip_msg.rotation.w = qw[0];
                    timing = 0;
                    // printf("position_delay = [%0.4f %0.4f %0.4f]\n", x[0], y[0], z[0]);
                }
                else
                {
                    tip_msg.translation.x = x[timing + 1];
                    tip_msg.translation.y = y[timing + 1];
                    tip_msg.translation.z = z[timing + 1];
                    tip_msg.rotation.x = qx[timing + 1];
                    tip_msg.rotation.y = qy[timing + 1];
                    tip_msg.rotation.z = qz[timing + 1];
                    tip_msg.rotation.w = qw[timing + 1];
                    // printf("position_delay = [%0.4f %0.4f %0.4f]\n", x[timing + 1], y[timing + 1], z[timing + 1]);
                    timing++;
                }
                pub_tip->publish(tip_msg);
            }

            if (!PUB_START)
            {
                PUB_START = true;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_logger, "tf2 error");
            return 0;
        }
        // Do some work in rclcpp and wait for more to come in.
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
