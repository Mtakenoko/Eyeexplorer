#ifndef POINTCLOUD_FILTER_HPP__
#define POINTCLOUD_FILTER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

class PointCloud_Filter : public rclcpp::Node
{
public:
    PointCloud_Filter();
    void topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hold_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    void add_hold_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliertemoval_filtering(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr paththrough_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    void publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    void show(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::visualization::CloudViewer viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hold_;

private:
    void PCL_to_PointCloud2msg(const pcl::PointCloud<pcl::PointXYZ> input_point, sensor_msgs::msg::PointCloud2 &msg_cloud_pub);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

#endif