#ifndef OCCUPANCY_GRIDMAP_HPP_
#define OCCUPANCY_GRIDMAP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <cv_bridge/cv_bridge.h>

#include <octomap/octomap.h>

#include <opencv2/opencv.hpp>

#include "/home/takeyama/workspace/htl/ros/msg_converter.hpp"
#include "/home/takeyama/workspace/htl/opencv/pose.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

#define fovx 198
#define fovy 198
#define u0 80.0
#define v0 80.0
#define WIDTH 160
#define HEIGHT 160
#define MIN_DEPTH 0.0
#define MAX_DEPTH 25.0

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
    subscription_depthimage = this->create_subscription<sensor_msgs::msg::Image>("/endoscope/image/depth", qos, std::bind(&Gridmap::topic_depthimage_callback_, this, std::placeholders::_1));

    // 占有グリッドの設定
    tree.setOccupancyThres(0.7);
    tree.setResolution(0.001);

    // カメラ内部パラメータの設定
    cv::Mat CameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0,
                            0.0, fovy, v0,
                            0.0, 0.0, 1.0);
    cv::Mat CameraPose(3, 4, CV_32FC1);
    cv::hconcat(cv::Mat::eye(3, 3, CV_32FC1), cv::Mat::zeros(3, 1, CV_32FC1), CameraPose);
    this->ProjectionMatrix = CameraMatrix * CameraPose;
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
    std::cout << std::endl;
    std::cout << "Received image" << std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_image);
    cv_ptr->image.convertTo(depth_image, CV_32F);
    // cv::imshow("depth", depth_image);
    // cv::waitKey(3);
    flag_setdepthimage = true;
}

void Gridmap::calc()
{
    // ワールド座標系から見た点群の位置を計算
    octomap::point3d origin(this->tip.position.x, this->tip.position.y, this->tip.position.z); // 計測原点。カメラの3次元座標。

    for (int i = 1; i < WIDTH - 1; i++)
    {
        for (int j = 1; j < HEIGHT - 1; j++)
        {
            htl::Position<float> point = this->get_Point3d(i, j);
            octomap::point3d end(point.x, point.y, point.z); // 計測した1点の3次元座標
            tree.updateNode(end, true);                      // integrate 'occupied' measurement
            // tree.insertRay(origin, end); // レイを飛ばして空間を削り出す
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
    float depth = (depth_image.at<float>(u, v) * (MAX_DEPTH - MIN_DEPTH) + MIN_DEPTH) / 1000.;

    float A1 = ProjectionMatrix.at<float>(0, 0) - ProjectionMatrix.at<float>(2, 0) * u;
    float B1 = ProjectionMatrix.at<float>(0, 1) - ProjectionMatrix.at<float>(2, 1) * u;
    float C1 = ProjectionMatrix.at<float>(0, 2) - ProjectionMatrix.at<float>(2, 2) * u;
    float D1 = ProjectionMatrix.at<float>(0, 4) - ProjectionMatrix.at<float>(2, 3) * u;
    float A2 = ProjectionMatrix.at<float>(1, 0) - ProjectionMatrix.at<float>(2, 0) * v;
    float B2 = ProjectionMatrix.at<float>(1, 1) - ProjectionMatrix.at<float>(2, 1) * v;
    float C2 = ProjectionMatrix.at<float>(1, 2) - ProjectionMatrix.at<float>(2, 2) * v;
    float D2 = ProjectionMatrix.at<float>(1, 3) - ProjectionMatrix.at<float>(2, 3) * v;

    htl::Position<float> point_camera;
    point_camera.x = -((C1 * B2 - C2 * B1) * depth + (D1 * B2 - D2 * B1)) / (A1 * B2 - A2 * B1);
    point_camera.y = -((C1 * A2 - C2 * A1) * depth + (D1 * A2 - D2 * A1)) / (B1 * A2 - B2 * A1);
    point_camera.z = depth;

    htl::Position<float> point_world;
    cv::Mat Rotation = htl::Transform::QuaternionToRotMat<float>(this->tip.quaternion);
    cv::Mat Transform = (cv::Mat_<float>(3, 1) << tip.position.x, tip.position.y, tip.position.z);
    cv::Mat temp_point = Rotation * cv::Mat1f(point_camera) + Transform;
    point_world.x = temp_point.at<float>(0);
    point_world.y = temp_point.at<float>(1);
    point_world.z = temp_point.at<float>(2);
    if (u == 10 && v == 20)
    {
        std::cout << "point_camera :" << point_camera << std::endl;
        std::cout << "point_world :" << point_world << std::endl;
    }
    return point_world;
}
#endif