// 順運動学の結果を補正し、正しいリンクパラメータ、オフセット角度を最適化により求める
// 位置が既知のマーカーに対して、
// 入力：ロボットの角度とリンクのパラメータ、内視鏡の画像、設置したマーカーの位置（既知なので別に入力じゃなくてもいいかも）
// 出力：角度オフセット量、リンクパラメータの推定値
// 課題：現在運動学を解くのは"urdf"+"robot_state_publisher"を用い

#ifndef CALIBRATE_FK_HPP__
#define CALIBRATE_FK_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ceres/ceres.h>
#include "cost_function.hpp"

struct Point_3D
{
    double x;
    double y;
    double z;
};

struct Point_image
{
    double u;
    double v;
};

struct Marker
{
    Point_3D Position;
    Point_image ProjectPoint;
};

struct Scene
{
    Marker marker;
    double joint[5];
    cv::Mat Image;
};

class Calib_Param
{
public:
    Calib_Param();
    void topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                         const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm);

private:
    void initialization();
    void process();
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_joint);
    void detect_marker(const cv::Mat &image, Marker *marker);
    void optimization();
    int encoding2mat_type(const std::string &encoding);

private:
    // マーカーの三次元位置および画像平面での位置およびその画像
    std::vector<Scene> scene;

    bool flag_take;
    bool flag_finish;
};

Calib_Param::Calib_Param() : flag_take(false), flag_finish(false)
{
}

void Calib_Param::topic_callback_(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                                  const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm)
{
    Calib_Param::input_data(msg_image, msg_arm);
    Calib_Param::optimization();
}

int Calib_Param::encoding2mat_type(const std::string &encoding)
{
    if (encoding == "mono8")
    {
        return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
        return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
        return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
        return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
        return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
        return CV_8UC3;
    }
    else
    {
        throw std::runtime_error("Unsupported encoding type");
    }
}

void Calib_Param::optimization()
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    // 最適化用パラメータ
    double mutable_link_param[21]; // リンクの長さ
    double mutable_angle_param[5]; // 角度オフセット量

    // リンクパラメータの初期値を代入(シーン毎に変化しない)
    mutable_link_param[0] = 9.;     // [mm]
    mutable_link_param[1] = 0.;     // [mm]
    mutable_link_param[2] = 32.;    // [mm]
    mutable_link_param[3] = 32.32;  // [mm]
    mutable_link_param[4] = 0.;     // [mm]
    mutable_link_param[5] = 17.68;  // [mm]
    mutable_link_param[6] = 200.;   // [mm]
    mutable_link_param[7] = 0.;     // [mm]
    mutable_link_param[8] = 0.;     // [mm]
    mutable_link_param[9] = 41.5;   // [mm]
    mutable_link_param[10] = 0.5;   // [mm]
    mutable_link_param[11] = -19.;  // [mm]
    mutable_link_param[12] = 0.;    // [mm]
    mutable_link_param[13] = 0.;    // [mm]
    mutable_link_param[14] = -200.; // [mm]
    mutable_link_param[15] = 94.9;  // [mm]
    mutable_link_param[16] = 0.;    // [mm]
    mutable_link_param[17] = 0.;    // [mm]
    mutable_link_param[18] = 0.;    // [mm]
    mutable_link_param[19] = 0.;    // [mm]
    mutable_link_param[20] = -110.; // [mm]

    // 角度量とコスト関数生成
    for (auto itr = scene.begin(); itr != scene.end(); itr++)
    {
        for (int i = 0; i < 5; i++)
        {
            mutable_angle_param[i] = itr->joint[i]; // [rad]
        }
        ceres::CostFunction *cost_function = NewProjectionErrorCostFuctor::Create(itr->marker.ProjectPoint.u, itr->marker.ProjectPoint.v,
                                                                                  itr->marker.Position.x, itr->marker.Position.y, itr->marker.Position.z);
        problem.AddResidualBlock(cost_function, NULL, mutable_link_param, mutable_angle_param);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

void Calib_Param::input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                             const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_joint)
{
    // 新規シーンのオブジェクト生成
    Scene new_scene;
    
    // 角度
    for (int i = 0; i < 5; i++)
    {
        new_scene.joint[i] = msg_joint->position[i];
    }

    // 画像
    cv::Mat frame_image(msg_image->height, msg_image->width, Calib_Param::encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    new_scene.Image = frame_image.clone();

    // マーカー位置検出
    Calib_Param::detect_marker(new_scene.Image, &new_scene.marker);

    scene.push_back(new_scene);
}

void Calib_Param::detect_marker(const cv::Mat &image, Marker *marker)
{
    // マーカーの種類判別

    // マーカーの位置検出

}

#endif