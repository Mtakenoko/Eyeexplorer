#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

//PCL Viewer
pcl::visualization::CloudViewer viewer("simple cloud viewer");
pcl::PointCloud<pcl::PointXYZ> cloud_hold;

void visualize_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr sub_msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    static uint32_t pre_width = 0, pre_height = 0;
    cloud.width = sub_msg->width + pre_width;
    cloud.height = sub_msg->height + pre_height;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);
    // printf("width = %d, height = %d\n", sub_msg->width, sub_msg->height);
    auto floatData = reinterpret_cast<float *>(sub_msg->data.data());
    for (uint32_t i = 0; i < sub_msg->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (sub_msg->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (sub_msg->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (sub_msg->point_step / sizeof(float)) + 2];
        cloud.points.push_back(xyz);
        printf("xyz = [%0.4f %0.4f %0.4f]\n", xyz.x, xyz.y, xyz.z);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    viewer.showCloud(cloud_ptr);
}

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("pointcloud");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("pcl_test");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    auto callback = [&node](const sensor_msgs::msg::PointCloud2::SharedPtr msg_sub) {
        visualize_pointcloud(msg_sub);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
