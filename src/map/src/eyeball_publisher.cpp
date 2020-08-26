#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <visualization_msgs/msg/marker.hpp>

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("eyeball_publisher");
    rclcpp::Logger node_logger = node->get_logger();

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Set quality of service profile based on command line options.
    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //Set QoS to Publish
    std::string topic_pub_eyeball("eyeball");
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_eyeball.c_str());
    auto pub_eyeball_marker = node->create_publisher<visualization_msgs::msg::Marker>(topic_pub_eyeball, qos); // Create the image publisher with our custom QoS profile.

    rclcpp::WallRate loop_rate(1);
    while (rclcpp::ok())
    {
        //set marker
        visualization_msgs::msg::Marker marker_msg;
        std::string ns = "test_ns";
        int id = 0;
        marker_msg.header.frame_id = "world";
        marker_msg.header.stamp = clock->now();
        marker_msg.ns = ns;
        marker_msg.id = ++id;

        marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
        marker_msg.action = visualization_msgs::msg::Marker::ADD;

        marker_msg.scale.x = 0.026;
        marker_msg.scale.y = 0.022;
        marker_msg.scale.z = 0.026;

        marker_msg.color.a = 0.2;
        marker_msg.color.r = 1.0;
        marker_msg.color.g = 1.0;
        marker_msg.color.b = 0.2;

        marker_msg.pose.position.x = 0.40058;
        marker_msg.pose.position.y = -0.1989;
        marker_msg.pose.position.z = 0.0000;

        //Publish
        RCLCPP_INFO(node->get_logger(), "Publishing Sphere");
        pub_eyeball_marker->publish(marker_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();

    return 0;
}