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

#define FOCAL_X 396.7
#define FOCAL_Y 396.9
#define U0 160
#define V0 160

// 現在のフレームと比較するKFを選択するためのパラメータ
#define CHOOSE_KF_Z_MAX 0.01
#define CHOOSE_KF_XY_MIN 0.005
#define CHOOSE_KF_XY_MAX 0.03
#define CHOOSE_KF_PHI_MIN 0.005
#define CHOOSE_KF_PHI_MAX 0.05

// 新しくKF挿入するためのパラメータ
// いっぱい取れるようにすると過去の分を使うことがなくなってしまうため、あまり一杯取らないように注意
#define SET_KF_Z_MAX 0.02
#define SET_KF_XY_MAX 0.03
#define SET_KF_PHI_MAX 0.05

// マッチング辞書の中からその特徴点がこの数より多いシーンで撮影されていることがわかれば三次元復元を行う
#define KEYPOINT_SCENE 4
#define KEYPOINT_SCENE_DELETE 10

// マッチング誤対応除去用パラメータ
#define THRESH_VARIANCE 100       // 分散のしきい値
#define THRESH_SMIROFF_GRUBBS 0.3 // スミルノフ･グラブス検定

// 3次元点の統計学的フィルタ用パラメータ
#define THRESH_VARIANCE_POINT 0.001       // 分散のしきい値

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
    cv::Mat point3D, point3D_BA, point3D_hold, point3D_BA_hold, point3D_filtered, point3D_filtered_hold;
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
    void triangulation_multiscene();
    void bundler();
    cv::Mat bundler_multiscene(const std::vector<MatchedData> &matchdata,
                               const cv::Mat &Point3D);
    void pointcloud_statics_filter(const cv::Mat &Point3D, cv::Mat *output_point3D);
    void setFirstFrame();
    void setKeyFrame();
    void chooseKeyFrame();
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