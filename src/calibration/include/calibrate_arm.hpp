// 順運動学の結果を補正し、正しいリンクパラメータ、オフセット角度を最適化により求める
// 位置が既知のマーカーに対して、
// 入力：ロボットの角度とリンクのパラメータ、内視鏡の画像、設置したマーカーの位置（既知なので別に入力じゃなくてもいいかも）
// 出力：角度オフセット量、リンクパラメータの推定値
// 課題：現在運動学を解くのは"urdf"+"robot_state_publisher"を用い

#ifndef CALIBRATE_FK_HPP_
#define CALIBRATE_FK_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

struct Marker
{
    cv::Point3f Position;
    cv::Point2f Point_Image;
    int ID;
    void setPosition(int marker_id);
};

struct Scene
{
    std::vector<Marker> marker;
    double joint[5];
    cv::Mat Image;
};

class Calib_Param
{
public:
    Calib_Param();
    void topic_callback_image_(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void topic_callback_joint_(const sensor_msgs::msg::JointState::SharedPtr msg_image);
    int getSceneNum();
    int getUseSceneNum();
    void getNowImage(cv::Mat *image);
    void getNewSceneImage(cv::Mat *image);
    void getNewMarkerImage(cv::Mat *image);
    bool getFinishFlag();
    void setCaptureFlag();
    void setCalibrationFlag();
    void saveOffsetData();
    void resetScene();
    void deleteScene();

private:
    void input_image_data(const sensor_msgs::msg::Image::SharedPtr msg_image);
    void input_joint_data(const sensor_msgs::msg::JointState::SharedPtr msg_image);
    void detect_marker(const cv::Mat &image, std::vector<Marker> *marker);
    void optimization();
    void optimization_link();
    void projectPoint();
    void clear();
    int encoding2mat_type(const std::string &encoding);
    void setNewScene();

private:
    // マーカーの三次元位置および画像平面での位置およびその画像
    std::vector<Scene> scene;
    Scene new_Scene;
    cv::Mat now_image;
    cv::Mat scene_image;
    cv::Mat marker_image;
    // マーカー検出用
    cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

private:
    double *offset_output;

public:
    int scene_counter;
    int use_scene_counter;

private:
    bool flag_set;
    bool flag_set_image;
    bool flag_set_joint;
    bool flag_finish;
    bool flag_optimize;
};
#endif