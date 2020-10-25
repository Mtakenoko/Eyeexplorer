#ifndef OCCUPANCY_GRIDMAP_HPP_
#define OCCUPANCY_GRIDMAP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <omp.h>

#include <octomap/octomap.h>

#include <opencv2/opencv.hpp>

#include "/home/takeyama/workspace/htl/ros/msg_converter.hpp"
#include "/home/takeyama/workspace/htl/opencv/pose.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

#define fovx 396.7
#define fovy 396.9
#define u0 163.6
#define v0 157.1
#define WIDTH 320
#define HEIGHT 320

class Gridmap : public rclcpp::Node
{
public:
    Gridmap();
    void set_VoxelMinimumSize(const double &size);
    void calc();
    void search();
    void publish();
    void writeMap();

private:
    void topic_tip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void topic_depthimage_callback_(const sensor_msgs::msg::Image::SharedPtr msg_depthimage);
    void input_tip_data(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void input_depthimage_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_tip_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depthimage;
    bool flag_settip;
    bool flag_setdepthimage;

private:
    htl::Pose<float> tip;
    cv::Mat depth_image;
    std::vector<htl::Position<float>> pointcloud;
    cv::Mat CameraMatrix;
    cv::Mat ProjectionMatrix;

private:
    void print_query_info(octomap::point3d query, octomap::OcTreeNode *node);
    htl::Position<float> get_Point3d(const int &width, const int &height);
    octomap::OcTree tree;
};

Gridmap::Gridmap() : Node("Gridmap_creator"),
                     flag_settip(false), flag_setdepthimage(false), tree(0.001)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_tip_ = this->create_subscription<geometry_msgs::msg::Transform>("/endoscope_transform", qos, std::bind(&Gridmap::topic_tip_callback_, this, std::placeholders::_1));
    subscription_depthimage = this->create_subscription<sensor_msgs::msg::Image>("/CNN_endoscope/image/depth", qos, std::bind(&Gridmap::topic_depthimage_callback_, this, std::placeholders::_1));

    // 占有グリッドの設定
    tree.setOccupancyThres(0.7);
    tree.setResolution(0.001);

    // カメラ内部パラメータの設定
    CameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0,
                    0.0, fovy, v0,
                    0.0, 0.0, 1.0);
}

void Gridmap::set_VoxelMinimumSize(const double &size)
{
    tree.setResolution(size);
}

void Gridmap::topic_tip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip)
{
    this->input_tip_data(msg_tip);
    if (flag_setdepthimage)
    {
        this->calc();
    }
}

void Gridmap::topic_depthimage_callback_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    this->input_depthimage_data(msg_image);
    if (flag_settip)
    {
        this->calc();
    }
}

void Gridmap::input_tip_data(const geometry_msgs::msg::Transform::SharedPtr msg_tip)
{
    tip.position.x = (float)msg_tip->translation.x;
    tip.position.y = (float)msg_tip->translation.y;
    tip.position.z = (float)msg_tip->translation.z;
    tip.quaternion.x = (float)msg_tip->rotation.x;
    tip.quaternion.y = (float)msg_tip->rotation.y;
    tip.quaternion.z = (float)msg_tip->rotation.z;
    tip.quaternion.w = (float)msg_tip->rotation.w;
    flag_settip = true;
}

void Gridmap::input_depthimage_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    cv::Mat frame_image(msg_image->height, msg_image->width, htl::Converter::Encoding_to_cvMattype(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    depth_image = frame_image.clone();
    flag_setdepthimage = true;
}

void Gridmap::calc()
{
    // ワールド座標系から見た点群の位置を計算
    octomap::point3d origin(this->tip.position.x, this->tip.position.y, this->tip.position.z); // 計測原点。カメラの3次元座標。
    cv::Mat Rotation = htl::Transform::QuaternionToRotMat<float>(this->tip.quaternion);
    cv::Mat Transform = (cv::Mat_<float>(3, 1) << tip.position.x, tip.position.y, tip.position.z);
    cv::Mat CameraPose(3, 4, CV_32FC1);
    cv::hconcat(Rotation.t(), -Rotation.t() * Transform, CameraPose);
    this->ProjectionMatrix = this->CameraMatrix * CameraPose;

    int i, j;
#pragma omp parallel for private(j)
    for (i = 0; i < WIDTH; i++)
    {
        for (j = 0; j < HEIGHT; j++)
        {
            htl::Position<float> point = this->get_Point3d(i, j);
            octomap::point3d end(point.x, point.y, point.z); // 計測した1点の3次元座標
            tree.insertRay(origin, end);                     // レイを飛ばして空間を削り出す
        }
    }

    flag_settip = false;
    flag_setdepthimage = false;
}

void Gridmap::search()
{
    octomap::OcTreeNode *result;
    octomap::point3d query(0., 0., 0.);
    result = tree.search(query);
    this->print_query_info(query, result);
}

void Gridmap::publish()
{
}

void Gridmap::writeMap()
{
    tree.writeBinary("src/test/output/simple_tree.bt");
    std::cout << "wrote example file simple_tree.bt" << std::endl
              << std::endl;
    std::cout << "now you can use octovis to visualize: octovis simple_tree.bt" << std::endl;
    std::cout << "Hint: hit 'F'-key in viewer to see the freespace" << std::endl
              << std::endl;
}

void Gridmap::print_query_info(octomap::point3d query, octomap::OcTreeNode *node)
{
    if (node != NULL)
    {
        std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
    }
    else
        std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;
}

htl::Position<float> Gridmap::get_Point3d(const int &u, const int &v)
{
    float depth = depth_image.at<float>(u, v);

    float P11 = this->ProjectionMatrix.at<float>(0, 0);
    float P12 = this->ProjectionMatrix.at<float>(0, 1);
    float P13 = this->ProjectionMatrix.at<float>(0, 2);
    float P14 = this->ProjectionMatrix.at<float>(0, 3);
    float P21 = this->ProjectionMatrix.at<float>(1, 0);
    float P22 = this->ProjectionMatrix.at<float>(1, 1);
    float P23 = this->ProjectionMatrix.at<float>(1, 2);
    float P24 = this->ProjectionMatrix.at<float>(1, 3);
    float P31 = this->ProjectionMatrix.at<float>(3, 0);
    float P32 = this->ProjectionMatrix.at<float>(3, 1);
    float P33 = this->ProjectionMatrix.at<float>(3, 2);
    float P34 = this->ProjectionMatrix.at<float>(3, 3);

    htl::Position<float> point;
    point.x = (((P14 - P34 * u) * (P22 - P32 * v) - (P24 - P34 * v) * (P12 - P32 * u)) + depth * ((P13 - P33 * u) * (P22 - P32 * v) - (P23 - P33 * v) * (P12 - P32 * u))) /
              ((P11 - P31 * u) * (P22 - P32 * v) - (P21 - P31 * v) * (P12 - P32 * u));
    point.y = (((P14 - P34 * u) * (P21 - P31 * v) - (P24 - P34 * v) * (P11 - P31 * u)) + depth * ((P13 - P22 * u) * (P21 - P31 * v) - (P23 - P33 * v) * (P11 - P31 * u))) /
              ((P12 - P32 * u) * (P21 - P31 * v) - (P22 - P32 * v) * (P11 - P31 * u));
    point.z = depth;
    return point;
}
#endif