#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "../include/map/pullout_endoscope.hpp"
#include "../option/options_pullout.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    float thresh_ransac = 0.01;
    int cpu_core = 8;
    size_t model = PullOut::PLANE_RANSAC;
    float safety_distance = 0.005;
    
    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy,
                               &thresh_ransac, &cpu_core, &model, &safety_distance))
        return 0;

    // Topic Name
    std::string topic_sub_pointcloud("pointcloud");
    std::string topic_sub_arm("endoscope_transform");
    std::string topic_pub("pointcloud2");
    auto pullout = PullOut();

    // PullOut の設定
    pullout.setModel(model);
    pullout.setThreshRANSAC(thresh_ransac);
    pullout.setSafetyDistance(safety_distance);

    // node
    auto node = rclcpp::Node::make_shared("pullout_endoscope");

    // コマンドラインでのQoSの設定
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // Pub/Subの設定
    auto publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub, qos);
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pointcloud_(node.get(), topic_sub_pointcloud);
    message_filters::Subscriber<geometry_msgs::msg::Transform> sub_arm_(node.get(), topic_sub_arm);
    message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::Transform> sync_(sub_pointcloud_, sub_arm_, 10000);
    sync_.registerCallback(std::bind(&PullOut::topic_callback_, pullout, std::placeholders::_1, std::placeholders::_2, publisher_));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}