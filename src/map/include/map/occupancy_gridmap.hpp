#ifndef OCCUPANCY_GRIDMAP_HPP_
#define OCCUPANCY_GRIDMAP_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <octomap/octomap.h>

#include <omp.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "/home/takeyama/workspace/htl/ros/msg_converter.hpp"
#include "/home/takeyama/workspace/htl/opencv/pose.hpp"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

// Camera Param
#define fovx 198
#define fovy 198
#define u0 80.0
#define v0 80.0
#define WIDTH 160
#define HEIGHT 160

// Depth Image Param
#define MIN_DEPTH 0.0
#define MAX_DEPTH 25.0

// Display Param
#define THRESHOLD_DISPLAY_OCC 0.5

class Gridmap : public rclcpp::Node
{
public:
    Gridmap();
    ~Gridmap();
    void set_OcTree_OccupancyThres(const double &thresh);
    void set_OcTree_Resolution(const double &resolution);
    void set_OcTree_ProbHit(const double &prob);
    void set_OcTree_ProbMiss(const double &prob);
    void set_OcTree_ClampingThresMax(const double &threshProb);
    void set_OcTree_ClampingThresMin(const double &threshProb);

private:
    void topic_tip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void topic_depthimage_callback_(const sensor_msgs::msg::Image::SharedPtr msg_depthimage);
    void input_tip_data(const geometry_msgs::msg::Transform::SharedPtr msg_tip);
    void input_depthimage_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void calc();
    void search(float x, float y, float z);
    void publish();
    void writeMap();
    void print_query_info(octomap::point3d query, octomap::OcTreeNode *node);
    htl::Position<float> get_Point3d(const int &width, const int &height);
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_tip_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_depthimage_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_markerarray_;

private:
    std::shared_ptr<octomap::OcTree> tree;
    htl::Pose<float> tip;
    cv::Mat depth_image;
    cv::Mat ProjectionMatrix;
    bool flag_set_tip;
    bool flag_set_depthimage;
};

Gridmap::Gridmap() : Node("Gridmap_creator"),
                     flag_set_tip(false), flag_set_depthimage(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_tip_ = this->create_subscription<geometry_msgs::msg::Transform>("/endoscope_transform", qos, std::bind(&Gridmap::topic_tip_callback_, this, std::placeholders::_1));
    subscription_depthimage_ = this->create_subscription<sensor_msgs::msg::Image>("/endoscope/image/depth", qos, std::bind(&Gridmap::topic_depthimage_callback_, this, std::placeholders::_1));
    publisher_markerarray_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupancy_grid/marker", qos);

    // カメラ内部パラメータの設定
    cv::Mat CameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0,
                            0.0, fovy, v0,
                            0.0, 0.0, 1.0);
    cv::Mat CameraPose(3, 4, CV_32FC1);
    cv::hconcat(cv::Mat::eye(3, 3, CV_32FC1), cv::Mat::zeros(3, 1, CV_32FC1), CameraPose);
    this->ProjectionMatrix = CameraMatrix * CameraPose;

    tree = std::make_shared<octomap::OcTree>(0.001);

    // // default Param
    // std::cout << std::endl;
    // std::cout << "OcTree default Param:" << std::endl
    //           << "  ClampingThresMax : " << tree->getClampingThresMax() << std::endl
    //           << "  ClampingThresMin : " << tree->getClampingThresMin() << std::endl
    //           << "  OccupancyThres : " << tree->getOccupancyThres() << std::endl
    //           << "  ProbHit : " << tree->getProbHit() << std::endl
    //           << "  ProbMiss : " << tree->getProbMiss() << std::endl
    //           << "  Resolution : " << tree->getResolution() << std::endl;
}

Gridmap::~Gridmap()
{
    this->writeMap();
}

void Gridmap::set_OcTree_OccupancyThres(const double &thresh)
{
    tree->setOccupancyThres(thresh);
}

void Gridmap::set_OcTree_ProbHit(const double &prob)
{
    tree->setProbHit(prob);
}

void Gridmap::set_OcTree_ProbMiss(const double &prob)
{
    tree->setProbMiss(prob);
}

void Gridmap::set_OcTree_ClampingThresMax(const double &threshProb)
{
    tree->setClampingThresMax(threshProb);
}

void Gridmap::set_OcTree_ClampingThresMin(const double &threshProb)
{
    tree->setClampingThresMin(threshProb);
}

void Gridmap::set_OcTree_Resolution(const double &resolution)
{
    tree->setResolution(resolution);
}

void Gridmap::topic_tip_callback_(const geometry_msgs::msg::Transform::SharedPtr msg_tip)
{
    this->input_tip_data(msg_tip);
    if (flag_set_depthimage)
        this->calc();
}

void Gridmap::topic_depthimage_callback_(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    this->input_depthimage_data(msg_image);
    if (flag_set_tip)
        this->calc();
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
    flag_set_tip = true;
}

void Gridmap::input_depthimage_data(const sensor_msgs::msg::Image::SharedPtr msg_image)
{
    std::cout << std::endl;
    std::cout << "Received image #" << msg_image->header.frame_id.c_str()<< std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_image);
    cv_ptr->image.convertTo(depth_image, CV_32F);
    flag_set_depthimage = true;
}

void Gridmap::calc()
{
    // ワールド座標系から見た点群の位置を計算
    octomap::point3d origin(this->tip.position.x, this->tip.position.y, this->tip.position.z); // 計測原点。カメラの3次元座標。
    // tree->updateNode(origin, false);

    htl::Position<float> test;
    for (int i = 1; i < WIDTH - 1; i++)
    {
        for (int j = 1; j < HEIGHT - 1; j++)
        {
            // printf("[%d, %d]\n", i, j);
            htl::Position<float> point = this->get_Point3d(i, j);
            // std::cout << "point : " << point << std::endl;
            octomap::point3d end(point.x, point.y, point.z); // 計測した1点の3次元座標1
            // tree->updateNode(end, true);                     // integrate 'occupied' measurement
            tree->insertRay(origin, end);             // レイを飛ばして空間を削り出す
            tree->insertRay(end + end - origin, end); // レイを飛ばして空間を削り出す(逆向きからも)
            if (i == 1 && j == 1)
                test = point;
        }
    }
    // this->search(0.0, 0.0, 0.0);
    // this->search(test.x, test.y, test.z);
    // this->search(test.x + 0.001, test.y, test.z);
    // this->search(test.x, test.y + 0.001, test.z);
    // this->search(test.x, test.y, test.z + 0.001);
    // this->search(test.x + 0.1, test.y + 0.1, test.z + 0.1);
    // this->search(this->tip.position.x, this->tip.position.y, this->tip.position.z);
    // std::cout << "tree->size() : " << this->tree->size() << std::endl;

    this->publish();

    flag_set_tip = false;
    flag_set_depthimage = false;
}

void Gridmap::search(float x, float y, float z)
{
    octomap::OcTreeNode *result;
    octomap::point3d query(x, y, z);
    result = tree->search(query);
    this->print_query_info(query, result);
}

void Gridmap::publish()
{
    auto markerarray_msgs_ = std::make_shared<visualization_msgs::msg::MarkerArray>();
    int id = 0;
    for (auto itr = tree->begin_leafs(); itr != tree->end_leafs(); itr++)
    {
        if (itr->getOccupancy() > THRESHOLD_DISPLAY_OCC)
        {
            visualization_msgs::msg::Marker marker_msg;
            rclcpp::Clock::SharedPtr clock = this->get_clock();

            marker_msg.header.frame_id = "world";
            marker_msg.header.stamp = clock->now();
            marker_msg.ns = "occupancy_grid";
            marker_msg.id = id++;

            // 形状
            marker_msg.type = visualization_msgs::msg::Marker::CUBE;
            marker_msg.action = visualization_msgs::msg::Marker::ADD;

            // 大きさ
            marker_msg.scale.x = 0.0005;
            marker_msg.scale.y = 0.0005;
            marker_msg.scale.z = 0.0005;

            // 色
            marker_msg.color.a = (float)itr->getOccupancy();
            marker_msg.color.r = 0.0;
            marker_msg.color.g = 0.0;
            marker_msg.color.b = 1.0;

            // 位置・姿勢
            marker_msg.pose.position.x = itr.getX();
            marker_msg.pose.position.y = itr.getY();
            marker_msg.pose.position.z = itr.getZ();
            marker_msg.pose.orientation.x = 0.0;
            marker_msg.pose.orientation.y = 0.0;
            marker_msg.pose.orientation.z = 0.0;
            marker_msg.pose.orientation.w = 1.0;

            // printf("position : [%lf %lf %lf] => %lf\n", itr.getX(), itr.getY(), itr.getZ(), itr->getOccupancy());

            markerarray_msgs_->markers.push_back(marker_msg);
        }
    }

    if (id != 0)
        this->publisher_markerarray_->publish(*markerarray_msgs_);
}

void Gridmap::writeMap()
{
    this->tree->writeBinary("/home/takeyama/workspace/ros2_eyeexplorer/src/map/output/simple_tree->bt");
    std::cout << std::endl;
    std::cout << "wrote example file simple_tree->bt" << std::endl
              << std::endl;
    std::cout << "now you can use octovis to visualize:" << std::endl
              << "  octovis /home/takeyama/workspace/ros2_eyeexplorer/src/map/output/simple_tree->bt" << std::endl;
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
    // if (u == 1 && v == 1)
    // {
    //     std::cout << "Rotation :" << Rotation << std::endl;
    //     std::cout << "Transform :" << cv::Point3f(Transform) << std::endl;
    //     std::cout << "point_camera :" << point_camera << std::endl;
    //     std::cout << "point_world :" << point_world << std::endl;
    // }
    return point_world;
}
#endif