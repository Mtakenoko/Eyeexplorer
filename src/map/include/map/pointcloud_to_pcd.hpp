#ifndef POINTCLOUD_TO_PCD_HPP__
#define POINTCLOUD_TO_PCD_HPP__

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

class PointCloud_to_PCD : public rclcpp::Node
{
public:
    PointCloud_to_PCD();
    void topic_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);
    void show(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    void save(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_);
    pcl::visualization::CloudViewer viewer;

private:
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    bool save_status, save_status2;
};

PointCloud_to_PCD::PointCloud_to_PCD()
    : Node("pointcloud_to_pcd"), viewer("cloud viewer"), save_status(false), save_status2(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/occupancy_grid/marker", qos, std::bind(&PointCloud_to_PCD::topic_callback_, this, std::placeholders::_1));
}

void PointCloud_to_PCD::topic_callback_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    // SubscribeしたものをPCL用データに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_ = PointCloud_to_PCD::input_data(msg_pointcloud);

    // RCLCPP_INFO
    RCLCPP_INFO(this->get_logger(), "cloud_->size() = %zu", cloud_->size());

    // 表示
    PointCloud_to_PCD::show(cloud_);

    if (cloud_->size() > 1500)
        this->save_status = true;

    // save
    PointCloud_to_PCD::save(cloud_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_to_PCD::input_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_(new pcl::PointCloud<pcl::PointXYZ>());

    input_cloud_->width = (int)msg_pointcloud->markers.size();
    input_cloud_->height = 1;
    input_cloud_->is_dense = false;
    // input_cloud_->resize(input_cloud_->width * input_cloud_->height);
    for (auto itr = msg_pointcloud->markers.begin(); itr != msg_pointcloud->markers.end(); itr++)
    {
        pcl::PointXYZ xyz;
        xyz.x = itr->pose.position.x;
        xyz.y = itr->pose.position.y;
        xyz.z = itr->pose.position.z;
        input_cloud_->points.push_back(xyz);
    }
    return input_cloud_;
}

void PointCloud_to_PCD::show(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_)
{
    viewer.showCloud(input_cloud_);
}

void PointCloud_to_PCD::save(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_)
{
    if (this->save_status && !this->save_status2)
    {
        this->save_status2 = true;
        std::cout << "saving PCD File 'ASCII'" << std::endl;
        pcl::io::savePCDFileASCII("/home/takeyama/workspace/ros2_eyeexplorer/src/map/output/cloud_ascii.pcd", *input_cloud_);
        std::cout << "saving PCD File 'Binary'" << std::endl;
        pcl::io::savePCDFileASCII("/home/takeyama/workspace/ros2_eyeexplorer/src/map/output/cloud_binary.pcd", *input_cloud_);
    }
}

#endif