#ifndef RECONSTRUCTION_HPP__
#define RECONSTRUCTION_HPP__

#include <map>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "CameraInfo.hpp"

#define FOCAL_X 396.7
#define FOCAL_Y 396.9
#define PP_X 160
#define PP_Y 160

#define IMAGE_WIDTH 320
#define IMAGE_HIGHT 320

// keyframe_databaseの初期登録数
#define KEYFRAME_DATABASE_NUM 3

// 現在のフレームと比較するKFを選択するためのパラメータ(一般用)
// 0.017 [PI]はで1[°]
#define CHOOSE_KF_Z_MAX_N 0.01
#define CHOOSE_KF_XY_MIN_N 0.01
#define CHOOSE_KF_XY_MAX_N 0.03
#define CHOOSE_KF_PHI_MAX_N 0.3
#define CHOOSE_KF_XY_MIN_2_N 0.005

// 新しくKF挿入するためのパラメータ(一般用)
// いっぱい取れるようにすると過去の分を使うことがなくなってしまうため、あまり一杯取らないように注意
#define SET_KF_Z_MAX_N 0.02
#define SET_KF_XY_MAX_N 0.03
#define SET_KF_PHI_MAX_N 0.05

// 現在のフレームと比較するKFを選択するためのパラメータ(眼球用)
#define CHOOSE_KF_Z_MAX_E 0.001
#define CHOOSE_KF_XY_MIN_E 0.0005
#define CHOOSE_KF_XY_MAX_E 0.001
#define CHOOSE_KF_PHI_MAX_E 0.05
#define CHOOSE_KF_XY_MIN_2_E 0.0001

// 新しくKF挿入するためのパラメータ(眼球用)
// いっぱい取れるようにすると過去の分を使うことがなくなってしまうため、あまり一杯取らないように注意
#define SET_KF_Z_MAX_E 0.001
#define SET_KF_XY_MAX_E 0.001
#define SET_KF_PHI_MAX_E 0.05

// マッチング辞書の中からその特徴点がこの数より多いシーンで撮影されていることがわかれば三次元復元を行う
#define KEYPOINT_SCENE 4
#define KEYPOINT_SCENE_DELETE 50

// マッチング誤対応除去用パラメータ
#define THRESH_VARIANCE 100       // 分散のしきい値
#define THRESH_SMIROFF_GRUBBS 0.3 // スミルノフ･グラブス検定

// ３次元点の距離フィルタ用パラメータ
#define THRESH_DISTANCE_EYE 0.02

// 3次元点の統計学的フィルタ用パラメータ
#define THRESH_VARIANCE_POINT 0.00005 // 分散のしきい値

// 移動量推定と運動学との差がおおきいときに三次元復元しないかどうか決定するパラメータ
#define THRESH_DOT 0.93

// ジンバルの中心から内視鏡先端までの距離
#define LENGTH_ENDOSCOPE 0.105
class Map
{
public:
    cv::Point3f point_3D;
    cv::Mat desciptors;
    int keypoiknt_id;
};

class PointData
{
public:
    cv::Mat point3D;
    cv::Point2f point2D;
};

class Reconstruction
{
public:
    Reconstruction();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal_hold,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA_hold,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered_hold,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est_hold,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_matching_image,
                         std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_nomatching_image,
                         std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_keyframe_marker,
                         std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_matchingframe_marker);
    void setThreshold_knn_ratio(float thresh);
    void setThreshold_ransac(float thresh);
    void setFlagShowImage(bool flag);
    void setFlagCeresstdout(bool flag);
    void setFlagEstimationMovement(bool flag);
    void setCPUCoreforBundler(int num);
    void setSceneNum(size_t num);
    void setUseMode(size_t num);
    void setMatchingMethod(size_t num);
    void setExtractor(size_t num);

    enum Matching
    {
        KNN = 0,
        BruteForce = 1
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
    void matching();
    void knn_matching();
    void BF_matching();
    void outlier_remover();
    void triangulate();
    void bundler();
    void pointcloud_eye_filter(const cv::Mat &InputPoint3D, cv::Mat *OutputPoint3D, const CameraInfo &camera_state);
    void setFirstFrame();
    void setKeyFrame();
    void setCameraInfo();
    void chooseKeyFrame();
    void updateKeyFrameDatabase();
    void managePointCloud();
    void keyframe_detector();
    void estimate_move();
    void process();
    void showImage();
    void publish(std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_normal_hold,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_BA_hold,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_filtered_hold,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud_est_hold,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_pointcloud_matching_image,
                 std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub_pointcloud_nomatching_image,
                 std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_keyframe_marker,
                 std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> pub_matchingframe_marker);
    std::string mat_type2encoding(int mat_type);
    void convert_frame_to_message(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg);
    void test();

private:
    // std::unique_ptr<FrameDatabase> frame_data = std::make_unique<FrameDatabase>();
    NowFrameData frame_data;
    KeyFrameData keyframe_data;
    std::vector<KeyFrameData> keyframe_database;
    std::vector<Map> map_point;
    cv::Mat point3D_hold,
        point3D_BA_hold,
        point3D_filtered_hold,
        point3D_est_hold;
    cv::Mat matching_image, nomatching_image;
    cv::Mat R_eye_move, t_eye_move;
    cv::Mat Rot_est, trans_est;

    const cv::Mat Rotation_eye = cv::Mat::eye(3, 3, CV_32F);
    const cv::Mat Transform_zeros = cv::Mat::zeros(3, 1, CV_32F);
    bool flag_reconstruction;
    bool flag_setFirstFrame;
    bool flag_showImage;
    bool flag_ceres_stdout;
    bool flag_estimate_move;
    bool flag_change_showImage;
    const cv::Mat CameraMat = (cv::Mat_<float>(3, 3) << FOCAL_X, 0.0, PP_X,
                               0.0, FOCAL_Y, PP_Y,
                               0.0, 0.0, 1.0);

    float threshold_knn_ratio;
    float threshold_ransac;
    int num_CPU_core;
    size_t num_Scene;
    size_t matching_method;
    int extract_type;
    size_t use_mode;
    float Z_MAX_CHOOSE, XY_MIN_CHOOSE, XY_MAX_CHOOSE, PHI_MAX_CHOOSE;
    float Z_MAX_SET, XY_MAX_SET, PHI_MAX_SET;
    float XY_MIN_2_CHOOSE;

    std::vector<cv::DMatch> dmatch, inliners_matches;
    std::vector<cv::Point2f> matched_point1, matched_point2;
    size_t match_num;
    std::vector<KeyFrameData>::iterator keyframe_itr;

    // バンドル調整用データコンテナ
    std::map<int, CameraInfo> camerainfo_map;    // keyはフレーム番号
    std::multimap<int, PointData> pointData_map; // keyはフレーム番号
    std::map<int, int> framenum_cam_map;         // keyはフレーム番号
    std::multimap<int, int> framenum_point_map;  // keyはフレーム番号
};
#endif