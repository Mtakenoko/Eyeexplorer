#include <cstdio>
#include <iostream>
#include <string>
#include <ktl.h>
#include <rclcpp/rclcpp.hpp>

#include "../include/msg_converter.h"
Converter::Converter()
{
}

std::string Converter::mat_type2encoding(int mat_type)
{
    switch (mat_type)
    {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

void Converter::cvimage_to_msg(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
{
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = Converter::mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}

void Converter::cvMat_to_msgPointCloud(const cv::Mat pointCloud, sensor_msgs::msg::PointCloud &msg_cloud_pub)
{
    msg_cloud_pub.header = std_msgs::msg::Header();
    msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";

    geometry_msgs::msg::Point32 point;
    for (int i = 0; i < pointCloud.rows; i++)
    {
        point.x = pointCloud.at<float>(i, 0);
        point.y = pointCloud.at<float>(i, 1);
        point.z = pointCloud.at<float>(i, 2);
        msg_cloud_pub.points.push_back(point);
    }
}

void Converter::cvMat_to_msgPointCloud2(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count)
{
    msg_cloud_pub.header = std_msgs::msg::Header();
    msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";

    msg_cloud_pub.is_bigendian = false;
    msg_cloud_pub.is_dense = true;

    msg_cloud_pub.height = 1;
    msg_cloud_pub.width = pointCloud2.rows;

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

    auto floatData = reinterpret_cast<float *>(msg_cloud_pub.data.data());
    for (uint32_t i = 0; i < msg_cloud_pub.width - dist_count; ++i)
    {
        for (uint32_t j = 0; j < 3; ++j)
        {
            floatData[i * (msg_cloud_pub.point_step / sizeof(float)) + j] = pointCloud2.at<cv::Vec3f>(i)[j];
        }
    }
}