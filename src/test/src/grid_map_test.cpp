#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/rcl.h"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;

//PCL Viewer
pcl::visualization::CloudViewer viewer("simple cloud viewer");
pcl::PointCloud<pcl::PointXYZ> cloud_hold;

void visualize_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr sub_msg,
                          std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> pub_map, nav_msgs::msg::OccupancyGrid msg,
                          rclcpp::Clock::SharedPtr clock)
{
    static uint32_t pre_width = 0;
    cloud_hold.width = sub_msg->width + pre_width;
    cloud_hold.height = sub_msg->height;
    cloud_hold.is_dense = false;
    cloud_hold.resize(cloud_hold.width * cloud_hold.height);
    printf("width = %d, height = %d\n", cloud_hold.width, cloud_hold.height);
    auto floatData = reinterpret_cast<float *>(sub_msg->data.data());
    for (uint32_t i = 0; i < sub_msg->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (sub_msg->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (sub_msg->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (sub_msg->point_step / sizeof(float)) + 2];
        cloud_hold.points.push_back(xyz);
        printf("xyz = [%0.4f %0.4f %0.4f]\n", xyz.x, xyz.y, xyz.z);
    }

    int lhs = 0;
    int center = 1;
    int rhs = 2;

    msg.data[(lhs) % (msg.info.width * msg.info.height)] = -1;
    msg.data[(center) % (msg.info.width * msg.info.height)] = -1;
    msg.data[(rhs) % (msg.info.width * msg.info.height)] = -1;
    msg.data[(++lhs) % (msg.info.width * msg.info.height)] = 0;
    msg.data[(++center) % (msg.info.width * msg.info.height)] = 100;
    msg.data[(++rhs) % (msg.info.width * msg.info.height)] = 0;

    msg.header.stamp = clock->now();

    pub_map->publish(msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_hold));
    viewer.showCloud(cloud_ptr);

    pre_width = cloud_hold.width;
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
    auto node = rclcpp::Node::make_shared("grid_map_test");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Time setting
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //map setting
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "world";
    msg.info.resolution = 0.1f;
    msg.info.width = 100;
    msg.info.height = 100;
    msg.info.origin.position.x = -(msg.info.width * msg.info.resolution) / 2;
    msg.info.origin.position.y = -(msg.info.width * msg.info.resolution) / 2;
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.info.origin.orientation.w = 1;
    for (size_t i = 0; i < msg.info.width * msg.info.height; ++i)
    {
        msg.data.push_back(-1);
    }

    //Publisher
    auto pub_map = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", qos);

    //CallBack関数
    auto callback = [pub_map, msg, clock, &node](const sensor_msgs::msg::PointCloud2::SharedPtr msg_sub) {
        visualize_pointcloud(msg_sub, pub_map, msg, clock);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
