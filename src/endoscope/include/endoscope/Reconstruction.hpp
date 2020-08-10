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

#define IMAGE_WIDTH 320
#define IMAGE_HIGHT 320
  
// keyframe_databaseの初期登録数
#define KEYFRAME_DATABASE_NUM 3

// 現在のフレームと比較するKFを選択するためのパラメータ(一般用)
#define CHOOSE_KF_Z_MAX_N 0.01
#define CHOOSE_KF_XY_MIN_N 0.008
#define CHOOSE_KF_XY_MAX_N 0.03
#define CHOOSE_KF_PHI_MIN_N 0.001
#define CHOOSE_KF_PHI_MAX_N 0.1

// 現在のフレームと比較するKFを選択するためのパラメータ(眼球用)
#define CHOOSE_KF_Z_MAX_E 0.01
#define CHOOSE_KF_XY_MIN_E 0.005
#define CHOOSE_KF_XY_MAX_E 0.01
#define CHOOSE_KF_PHI_MIN_E 0.005
#define CHOOSE_KF_PHI_MAX_E 0.05

// // 新しくKF挿入するためのパラメータ(一般用)
// // いっぱい取れるようにすると過去の分を使うことがなくなってしまうため、あまり一杯取らないように注意
#define SET_KF_Z_MAX_N 0.02
#define SET_KF_XY_MAX_N 0.03
#define SET_KF_PHI_MAX_N 0.05

// 新しくKF挿入するためのパラメータ(眼球用)
#define SET_KF_Z_MAX_E 0.01
#define SET_KF_XY_MAX_E 0.03
#define SET_KF_PHI_MAX_E 0.03

// マッチング辞書の中からその特徴点がこの数より多いシーンで撮影されていることがわかれば三次元復元を行う
#define KEYPOINT_SCENE 4
#define KEYPOINT_SCENE_DELETE 10

// マッチング誤対応除去用パラメータ
#define THRESH_VARIANCE 100       // 分散のしきい値
#define THRESH_SMIROFF_GRUBBS 0.3 // スミルノフ･グラブス検定

// 3次元点の統計学的フィルタ用パラメータ
#define THRESH_VARIANCE_POINT 0.001 // 分散のしきい値

// 移動量推定と運動学との差がおおきいときに三次元復元しないかどうか決定するパラメータ
#define THRESH_DOT 0.93

class Map
{
public:
    cv::Point3f point_3D;
    cv::Mat desciptors;
    int keypoiknt_id;
};

class Reconstruction
{
public:
    Reconstruction();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);
    void setThreshold_knn_ratio(float thresh);
    void setThreshold_ransac(float thresh);
    void setFlagShowImage(bool flag);
    void setFlagEstimationMovement(bool flag);
    void setCPUCoreforBundler(int num);
    void setSceneNum(size_t num);
    void setPublishType(size_t num);
    void setUseMode(size_t num);

    enum Matching
    {
        KNN = 0,
        BruteForce = 1
    };

    enum Publish
    {
        NORMAL = 0,
        NORMAL_HOLD = 1,
        BUNDLE = 2,
        BUNDLE_HOLD = 3,
        FILTER = 4,
        FILTER_HOLD = 5,
        ESTIMATE = 6,
        ESTIMATE_HOLD = 7
    };

    enum UseMode
    {
        NORMAL_SCENE = 0,
        EYE = 1
    };

private:
    void initialize();
    int encoding2mat_type(const std::string &encoding);
    std::vector<cv::Point2f> keypoint2Point(std::vector<cv::KeyPoint> kp);
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm);
    void knn_matching();
    void BF_matching();
    void outlier_remover();
    void triangulate();
    cv::Mat bundler(const std::vector<MatchedData> &matchdata, const cv::Mat &Point3D);
    bool pointcloud_statics_filter(const cv::Mat &Point3D, cv::Mat *output_point3D);
    void setFirstFrame();
    void setKeyFrame();
    void setCameraInfo();
    void chooseKeyFrame();
    void keyframe_detector();
    void estimate_move();
    void process();
    void manageMap();
    void registMap(const cv::Mat &point3D_);
    void checkMapPoint();
    void showImage();
    void publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);

private:
    // std::unique_ptr<FrameDatabase> frame_data = std::make_unique<FrameDatabase>();
    FrameDatabase frame_data;
    FrameDatabase keyframe_data;
    std::vector<FrameDatabase> keyframe_database;
    std::vector<Map> map_point;
    cv::Mat point3D, point3D_hold,
        point3D_BA, point3D_BA_hold,
        point3D_filtered, point3D_filtered_hold,
        point3D_est, point3D_est_hold;
    cv::Mat matching_image, nomatching_image;
    cv::Mat R_eye_move, t_eye_move;
    cv::Mat Rot_est, trans_est;

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
    int num_CPU_core;
    size_t num_Scene;
    size_t matching;
    int extract_type;
    size_t publish_type;
    size_t use_mode;
    std::vector<std::vector<cv::DMatch>> knn_matches;
    std::vector<cv::DMatch> dmatch, inliners_matches;
    std::vector<cv::Point2f> matched_point1, matched_point2;
    size_t match_num;
    cv::Mat cameraMatrix;
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    std::vector<FrameDatabase>::iterator keyframe_itr;
};
#endif