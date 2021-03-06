// 順運動学の結果を補正し、正しいリンクパラメータ、オフセット角度を最適化により求める
// 位置が既知のマーカーに対して、
// 入力：ロボットの角度とリンクのパラメータ、内視鏡の画像、設置したマーカーの位置（既知なので別に入力じゃなくてもいいかも）
// 出力：角度オフセット量、リンクパラメータの推定値
// 課題：現在運動学を解くのは"urdf"+"robot_state_publisher"を用い

#ifndef CALIBRATE_DELAY_HPP_
#define CALIBRATE_DELAY_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "Scene.hpp"

#define SCENE_NUM 35
#define DELAY_TIME 100

class Calib_Param
{
public:
    Calib_Param();
    void topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void topic_callback_joint_(const sensor_msgs::msg::JointState::SharedPtr msg_image);
    int getSceneNum();
    int getUseSceneNum();
    void getNowImage(cv::Mat *image);
    void getNewMarkerImage(cv::Mat *image);
    bool getFinishFlag();
    void setCalibrationFlag();
    void setStartFlag();
    void saveOffsetData();

private:
    void input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void input_joint_data(const sensor_msgs::msg::JointState::SharedPtr msg_image);
    bool detect_marker(const cv::Mat &image, std::vector<Marker> *marker);
    void optimization_without_ceres();
    void drawReprojectedMarkerPoint(const Scene &scene);
    void clear();
    int encoding2mat_type(const std::string &encoding);
    void setNewScene();
    void decideHandleScene();

private:
    // マーカーの三次元位置および画像平面での位置およびその画像
    std::vector<Scene> scene;
    Scene new_Scene[SCENE_NUM];
    int handle_scene;
    cv::Mat now_image;
    cv::Mat marker_image;
    // マーカー検出用
    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

private:
    double *offset_output;
    int delay;

public:
    int scene_counter;
    int use_scene_counter;

private:
    bool flag_finish;
    bool flag_optimize;
    bool flag_start;
};
#endif