#ifndef POINTCLOUD_FILTER_HPP__
#define POINTCLOUD_FILTER_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
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

PointCloud_Filter::PointCloud_Filter()
    : Node("eyeball_estimator"), viewer("cloud viewer"),
      cloud_hold_(new pcl::PointCloud<pcl::PointXYZ>())
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", qos,
        std::bind(&PointCloud_Filter::topic_callback_, this, std::placeholders::_1));
}

void PointCloud_Filter::topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    // SubscribeしたものをPCL用データに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_ = PointCloud_Filter::input_data(msg_pointcloud);
    PointCloud_Filter::hold_data(msg_pointcloud);

    // // フィルタリング
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    // filtered_cloud_ = PointCloud_Filter::outliertemoval_filtering(cloud_);

    // // 点群を保持
    // PointCloud_Filter::add_hold_point(filtered_cloud_);

    // フィルタリング
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    filtered_cloud_ = PointCloud_Filter::outliertemoval_filtering(cloud_hold_);

    // Publish用の点群ポインタ
    // pcl::PointCloud<pcl::PointXYZ>::Ptr publish_cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    // cloud_hold_ = PointCloud_Filter::voxelgrid_filter(cloud_hold_);

    // 表示
    PointCloud_Filter::show(cloud_hold_);

    // Publish
    PointCloud_Filter::publish(filtered_cloud_);

    // RCLCPP_INFO
    RCLCPP_INFO(this->get_logger(), "cloud_->width = %zu", cloud_->width);
    RCLCPP_INFO(this->get_logger(), "cloud_hold_->width = %zu", cloud_hold_->width);
    RCLCPP_INFO(this->get_logger(), "filtered_cloud_->width = %zu", filtered_cloud_->width);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Filter::input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_->width = msg_pointcloud->width;
    cloud_->height = msg_pointcloud->height;
    cloud_->is_dense = false;
    cloud_->resize(cloud_->width * cloud_->height);
    const auto floatData = reinterpret_cast<float *>(msg_pointcloud->data.data());
    for (uint32_t i = 0; i < msg_pointcloud->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 2];
        cloud_->points.push_back(xyz);
    }
    return cloud_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Filter::hold_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    cloud_hold_->width += msg_pointcloud->width;
    cloud_hold_->height = msg_pointcloud->height;
    cloud_hold_->is_dense = false;
    cloud_hold_->resize(cloud_hold_->width * cloud_hold_->height);

    auto floatData = reinterpret_cast<float *>(msg_pointcloud->data.data());
    for (uint32_t i = 0; i < msg_pointcloud->width; ++i)
    {
        pcl::PointXYZ xyz;
        xyz.x = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 0];
        xyz.y = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 1];
        xyz.z = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + 2];
        cloud_hold_->points.push_back(xyz);
    }
    return cloud_hold_;
}

void PointCloud_Filter::add_hold_point(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    cloud_hold_->width += input_point_->width;
    cloud_hold_->height = input_point_->height;
    cloud_hold_->is_dense = false;
    cloud_hold_->resize(cloud_hold_->width * cloud_hold_->height);
    for (uint32_t i = 0; i < input_point_->width; i++)
    {
        cloud_hold_->points.push_back(input_point_->points[i]);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Filter::outliertemoval_filtering(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_point_(new pcl::PointCloud<pcl::PointXYZ>());

    //フィルタオブジェクトの作成
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_point_);
    sor.setMeanK(8);
    sor.setStddevMulThresh(-(5.0e-2));
    sor.filter(*output_point_);
    sor.getRemovedIndices();

    return output_point_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Filter::paththrough_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_point_(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input_point_);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 10);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0, 10);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 10);
    // pass.setFilterLimitsNegative (true);
    pass.filter(*output_point_);

    return output_point_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud_Filter::voxelgrid_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_point_(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::VoxelGrid<pcl::PointXYZ> voxel_;
    voxel_.setLeafSize(0.001f, 0.001f, 0.001f); // 0.001[m] = 1[mm] 間隔でダウンサンプリング
    voxel_.setInputCloud(input_point_);
    voxel_.filter(*output_point_);

    return output_point_;
}

void PointCloud_Filter::show(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    viewer.showCloud(input_point_);
}

void PointCloud_Filter::PCL_to_PointCloud2msg(const pcl::PointCloud<pcl::PointXYZ> input_cloud, sensor_msgs::msg::PointCloud2 &msg_cloud_pub)
{
    msg_cloud_pub.header = std_msgs::msg::Header();
    msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";
    msg_cloud_pub.is_bigendian = false;
    msg_cloud_pub.is_dense = true;
    msg_cloud_pub.height = 1;
    msg_cloud_pub.width = input_cloud.width;
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
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 0] = input_cloud.points.at(i).x;
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 1] = input_cloud.points.at(i).y;
        floatData2[i * (msg_cloud_pub.point_step / sizeof(float)) + 2] = input_cloud.points.at(i).z;
    }
}

void PointCloud_Filter::publish(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_)
{
    auto msg_cloud_pub_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
    PointCloud_Filter::PCL_to_PointCloud2msg(*input_point_, *msg_cloud_pub_);
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
    publisher_->publish(std::move(msg_cloud_pub_));
}

#endif