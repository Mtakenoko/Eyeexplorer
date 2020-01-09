#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ktl.h>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

sensor_msgs::msg::PointCloud2 msg_storage_pcl;
void storage_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr sub_msg,
                        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    static uint32_t pre_width = 0;
    msg_storage_pcl.header = std_msgs::msg::Header();
    msg_storage_pcl.header.stamp = rclcpp::Clock().now();
    msg_storage_pcl.header.frame_id = "world";

    msg_storage_pcl.is_bigendian = false;
    msg_storage_pcl.is_dense = true;

    msg_storage_pcl.height = 1;
    pre_width = msg_storage_pcl.width;
    msg_storage_pcl.width = pre_width + sub_msg->width;

    // msg_storage_pcl.fields.resize(3);
    msg_storage_pcl.fields[0].name = "x";
    msg_storage_pcl.fields[1].name = "y";
    msg_storage_pcl.fields[2].name = "z";

    sensor_msgs::msg::PointField::_offset_type offset = 0;
    for (uint32_t i = 0; i < msg_storage_pcl.fields.size(); ++i, offset += sizeof(float))
    {
        msg_storage_pcl.fields[i].count = 1;
        msg_storage_pcl.fields[i].offset = offset;
        msg_storage_pcl.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    msg_storage_pcl.point_step = offset;
    msg_storage_pcl.row_step = msg_storage_pcl.point_step * msg_storage_pcl.width;

    auto floatData = reinterpret_cast<float *>(msg_storage_pcl.data.data());
    for (uint32_t i = pre_width; i < msg_storage_pcl.width; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            floatData[i * (msg_storage_pcl.point_step / sizeof(float)) + j] = floatData[(i - pre_width) * (msg_storage_pcl.point_step / sizeof(float)) + j];
        }
    }

    pub_pointcloud->publish(msg_storage_pcl);
}

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    bool show_camera = false;

    std::string topic_sub_pointcloud("pointcloud");
    std::string topic_pub_pointcloud("pointcloud_storage");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (show_camera)
    {
        // Initialize an OpenCV named window called "cvframe".
        cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
    }
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("pointcloud_storage");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    auto callback = [pub_pointcloud](const sensor_msgs::msg::PointCloud2::SharedPtr msg_sub) {
        storage_pointcloud(msg_sub, pub_pointcloud);
    };
    auto sub_arm = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic_sub_pointcloud, qos, callback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}