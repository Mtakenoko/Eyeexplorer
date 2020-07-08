#ifndef RECONSTRUCTION_HPP__
#define RECONSTRUCTION_HPP__

#include <map>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "CameraInfo.hpp"
#include "Bundler.hpp"
#include "cost_function.hpp"

#define FOCAL_X 396.7
#define FOCAL_Y 396.9
#define U0 160
#define V0 160
#define TRI_ITERATIVE_TERM 10

class Reconstruction
{
public:
    Reconstruction();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);

public:
    // std::unique_ptr<FrameDatabase> frame_data = std::make_unique<FrameDatabase>();
    FrameDatabase frame_data;
    FrameDatabase keyframe_data;
    std::vector<FrameDatabase> keyframe_database;
    cv::Mat point3D, point3D_BA;
    cv::Mat matching_image;

private:
    void initialize();
    int encoding2mat_type(const std::string &encoding);
    std::vector<cv::Point2f> keypoint2Point(std::vector<cv::KeyPoint> kp);
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm);
    void knn_matching();
    void BF_matching();
    void knn_outlier_remover();
    void BF_outlier_remover();
    
    void triangulation();
    void mappingKeyPoint();
    void triangulation_est();
    void triangulation_test();
    void triangulation_multiscene();
    void bundler();
    void setFirstFrame();
    void setKeyFrame();
    void chooseKeyFrame();
    bool checkKeyFrame();
    void keyframe_detector();
    void estimate_move();
    void process();
    void showImage();
    void publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);

private:
    const cv::Mat Rotation_eye = cv::Mat::eye(3, 3, CV_32F);
    const cv::Mat Transform_zeros = cv::Mat::zeros(3, 1, CV_32F);
    bool flag_reconstruction;
    bool flag_setFirstFrame;
    bool flag_showImage;
    bool flag_estimate_move;
    const cv::Mat CameraMat = (cv::Mat_<float>(3, 3) << FOCAL_X, 0.0, U0,
                               0.0, FOCAL_Y, V0,
                               0.0, 0.0, 1.0);

    float threshold_knn_ratio;
    float threshold_ransac;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> dmatch, inliners_matches;
    std::vector<cv::Point2f> matched_point1, matched_point2;
    size_t match_num;
    cv::Mat cameraMatrix;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat R_est, t_est;
    std::vector<FrameDatabase>::iterator keyframe_itr;
};
#endif