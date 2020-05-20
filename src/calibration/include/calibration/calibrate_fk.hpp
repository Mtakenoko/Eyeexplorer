// 順運動学の結果を補正し、正しいリンクパラメータ、オフセット角度を最適化により求める
// 位置が既知のマーカーに対して、
// 入力：ロボットの角度とリンクのパラメータ、内視鏡の画像、設置したマーカーの位置（既知なので別に入力じゃなくてもいいかも）
// 出力：角度オフセット量、リンクパラメータの推定値
// 課題：現在運動学を解くのは"urdf"+"robot_state_publisher"を用い

#ifndef CALIBRATE_FK_HPP__
#define CALIBRATE_FK_HPP__

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ceres/ceres.h>
#include "ceres_param.hpp"

class Marker
{
public:
    Marker(){};
    cv::Mat Position;  
    cv::Mat ProjectPoint;
};

class Calib_Param
{
public:
    Calib_Param();
    void topic_callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                        const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm,
                        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud);
    double link_param[10];
    double rad_offset[5];

private:
    void initialization();
    void process();
    void input_data(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image,
                    const std::shared_ptr<const sensor_msgs::msg::JointState> &msg_arm);
    void optimization();
    void publish();
    Marker marker[5];
};

Calib_Param::Calib_Param()
    : link_param{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}
{
}

void Calib_Param::optimization()
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;

    // 最適化用パラメータ(3個の視点で)
    double mutable_link_param[10]; // リンクの長さ
    double mutable_angle_param[5]; // 角度オフセット量

    // リンクパラメータの初期値を代入
    mutable_link_param[0] = 100.; // [mm]
    mutable_link_param[1] = 100.; // [mm]
    mutable_link_param[2] = 100.; // [mm]
    mutable_link_param[3] = 100.; // [mm]
    mutable_link_param[4] = 100.; // [mm]
    mutable_link_param[5] = 100.; // [mm]
    mutable_link_param[6] = 100.; // [mm]
    mutable_link_param[7] = 100.; // [mm]
    mutable_link_param[8] = 100.; // [mm]
    mutable_link_param[9] = 100.; // [mm]

    // 角度オフセット量
    mutable_angle_param[0] = 0.; // [rad]
    mutable_angle_param[1] = 0.; // [rad]
    mutable_angle_param[2] = 0.; // [rad]
    mutable_angle_param[3] = 0.; // [rad]
    mutable_angle_param[4] = 0.; // [rad]

    int scene_num = 5;
    for (int i = 0; i < scene_num; i++)
    {
        ceres::CostFunction *cost_function = SnavelyReprojectionError::Create(marker[i].ProjectPoint.at<double>[0], marker[i].ProjectPoint.at<double>[1]);
        problem.AddResidualBlock(cost_function, NULL, mutable_link_param, mutable_angle_param);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}
#endif