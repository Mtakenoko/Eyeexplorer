// 順運動学の結果を補正し、正しいリンクパラメータ、オフセット角度を最適化により求める
// 位置が既知のマーカーに対して、
// 入力：ロボットの角度とリンクのパラメータ、内視鏡の画像、設置したマーカーの位置（既知なので別に入力じゃなくてもいいかも）
// 出力：角度オフセット量、リンクパラメータの推定値
// 課題：現在運動学を解くのは"urdf"+"robot_state_publisher"を用い

#ifndef CALIBRATE_FK_HPP_
#define CALIBRATE_FK_HPP_

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

struct Marker_dataset
{
};

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
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm);
    int getSceneNum();
    cv::Mat getNewSceneImage();

private:
    void initialization();
    void process();
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_joint);
    void detect_marker(const cv::Mat &image, std::vector<Marker> *marker);
    void optimization();
    int encoding2mat_type(const std::string &encoding);

private:
    // マーカーの三次元位置および画像平面での位置およびその画像
    std::vector<Scene> scene;

public:
    int scene_counter;
    bool flag_set;
    bool flag_finish;
    bool flag_optimize;
};
#endif