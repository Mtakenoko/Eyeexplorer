#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/transform.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_msgs/msg/tf_message.hpp>
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
    auto node = rclcpp::Node::make_shared("arm_state_publisher");
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

    // lanuchファイルで立ち上げる時にまだ準備できていないときがあるので、少し待つ
    sleep(1);

    while (rclcpp::ok())
    {
        try
        {
            geometry_msgs::msg::TransformStamped TransformStamped;
            TransformStamped = buffer_.lookupTransform(target_frame, source_frame, tf2::TimePoint());

            geometry_msgs::msg::Transform tip_msg;
            tip_msg.translation.x = TransformStamped.transform.translation.x;
            tip_msg.translation.y = TransformStamped.transform.translation.y;
            tip_msg.translation.z = TransformStamped.transform.translation.z;
            tip_msg.rotation.x = TransformStamped.transform.rotation.x;
            tip_msg.rotation.y = TransformStamped.transform.rotation.y;
            tip_msg.rotation.z = TransformStamped.transform.rotation.z;
            tip_msg.rotation.w = TransformStamped.transform.rotation.w;
            pub_tip->publish(tip_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(node_logger, "tf2 error: %s", ex.what());
            // return 0;
        }
        // Do some work in rclcpp and wait for more to come in.
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
