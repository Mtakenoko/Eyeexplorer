#ifndef POINTCLOUD_TO_PCD_HPP__
#define POINTCLOUD_TO_PCD_HPP__

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/visualization/cloud_viewer.h>

class PointCloud_to_PCD
{
public:
    PointCloud_to_PCD();
    void topic_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);
    void showPointClooud();
    void showSavedPCD();
    void savePCD();
    int getPointCloudNum();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);
    void save(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::visualization::CloudViewer viewer;
    // pcl::visualization::CloudViewer saved_viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr saved_cloud_;
};
#endif