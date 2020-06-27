#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../include/endoscope/Reconstruction.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Ceres Solver Logging用　Initializer
    google::InitGoogleLogging(argv[0]);

    // Topic Name
    std::string topic_sub_track("endoscope_image");
    std::string topic_sub_arm("endoscope_transform");
    std::string topic_pub("pointcloud");
    auto reconstructor = Reconstruction();

    // node
    auto node = rclcpp::Node::make_shared("reconstructor"); //Set QoS to Publish

    // Set quality of service profile based on command line options.
    // コマンドラインでのQoSの設定（よくわからん）
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Pub/Subの設定
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    auto publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub, qos);
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_track_(node.get(), topic_sub_track);
    message_filters::Subscriber<geometry_msgs::msg::Transform> sub_arm_(node.get(), topic_sub_arm);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync_(sub_track_, sub_arm_, 1000);
    sync_.registerCallback(std::bind(&Reconstruction::topic_callback_, reconstructor, std::placeholders::_1, std::placeholders::_2, publisher_));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}