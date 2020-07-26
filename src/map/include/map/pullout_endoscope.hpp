#ifndef PULLOUT_ENDOSCOPE_HPP__
#define PULLOUT_ENDOSCOPE_HPP__

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "../../../endoscope/include/endoscope/CameraInfo.hpp"

#define RANSAC_DISTANCE_THRESHOLD 0.01
#define SAFETY_DISTANCE 0.005 //[m]

class EndoscopePose
{
public:
    Eigen::Matrix3f Rotation;
    Eigen::Matrix<float, 3, 1> Transform;
};

class PullOut
{
public:
    PullOut();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg_pointcloud,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud);

private:
    void initialize();
    bool input_data(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg_pointcloud,
                    const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_,
                    EndoscopePose *endoscopePose);
    void process(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                 const EndoscopePose endoscopePose);
    void publish(const std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> &pub_pointcloud);

    bool chace_empty(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    void estimate_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float *OutputCoefficients);
    void estimate_plane_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, float *OutputCoefficients);
    void estimate_sphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    float calc_distance(const float *coefficients, const EndoscopePose endoscopePose); // 平面のパラメータと内視鏡の位置から、内視鏡と平面の距離を測定する

private:
    bool flag_pull;
};

#endif