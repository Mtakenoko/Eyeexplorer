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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std::chrono_literals;
pcl::visualization::CloudViewer viewer("cloud viewer");
pcl::PointCloud<pcl::PointXYZ> cloud_hold;

void PCL_to_PCLmsg(pcl::PointCloud<pcl::PointXYZ> cloud_ptr, sensor_msgs::msg::PointCloud2 &msg_cloud_pub)
{
    msg_cloud_pub.header = std_msgs::msg::Header();
    msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";
    msg_cloud_pub.is_bigendian = false;
    msg_cloud_pub.is_dense = true;
    msg_cloud_pub.height = 1;
    msg_cloud_pub.width = cloud_ptr.width;
    msg_cloud_pub.fields.resize(3);
    msg_cloud_pub.fields[0].name = "x";
    msg_cloud_pub.fields[1].name = "y";
    msg_cloud_pub.fields[2].name = "z";
    sensor_msgs::msg::PointField::_offset_type offset = 0;
    for (uint32_t i = 0; i < msg_cloud_pub.fields.size(); ++i, offset += sizeof(float))
    {
        msg_cloud_pub.fields[i].count = 1;
        msg_cloud_pub.fields[i].offset = offset;
        msg_cloud_pub.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }
    msg_cloud_pub.point_step = offset;
    msg_cloud_pub.row_step = msg_cloud_pub.point_step * msg_cloud_pub.width;
    msg_cloud_pub.data.resize(msg_cloud_pub.row_step * msg_cloud_pub.height);
    auto floatData2 = reinterpret_cast<float *>(msg_cloud_pub.data.data());

    for (uint32_t i = 0; i < msg_cloud_pub.width; ++i)
    {
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 0] = cloud_ptr.points.at(i).x;
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 1] = cloud_ptr.points.at(i).y;
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 2] = cloud_ptr.points.at(i).z;
    }
}

void outlier_remover(const sensor_msgs::msg::PointCloud2::SharedPtr &sub_msg,
                          std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = sub_msg->width;
    cloud.height = sub_msg->height;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    static uint32_t pre_width = 0;
    cloud_hold.width = sub_msg->width + pre_width;
    cloud_hold.height = sub_msg->height;
    cloud_hold.is_dense = false;
    cloud_hold.resize(cloud_hold.width * cloud_hold.height);
    auto floatData = reinterpret_cast<float *>(sub_msg->data.data());
    for (uint32_t i = 0; i < sub_msg->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (sub_msg->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (sub_msg->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (sub_msg->point_step / sizeof(float)) + 2];
        cloud.points.push_back(xyz);
        cloud_hold.points.push_back(xyz);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hold_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud_hold));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    //フィルタオブジェクトの作成
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_hold_ptr);
    sor.setMeanK(8);
    sor.setStddevMulThresh(-(5.0e-2));
    sor.filter(*cloud_filtered_ptr);
    sor.getRemovedIndices();

    //Viewer
    viewer.showCloud(cloud_filtered_ptr);

    // Publish
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    PCL_to_PCLmsg(*cloud_hold_ptr, *msg_cloud_pub);
    pub_pointcloud->publish(std::move(msg_cloud_pub));

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
    std::string topic_pub_pointcloud("pointcloud_filtered");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("pcl_outlier_remove");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    auto callback = [pub_pointcloud, &node](const sensor_msgs::msg::PointCloud2::SharedPtr msg_sub) {
        outlier_remover(msg_sub, pub_pointcloud);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
