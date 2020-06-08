#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/rotation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ktl.h>

#include "../include/arm.hpp"
#include "../include/cost_function.hpp"
#include "../../HTL/include/transform.h"

NewProjectionErrorCostFuctor::NewProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                                                           double marker_x_, double marker_y_, double marker_z_,
                                                           double angle0_, double angle1_, double angle2_, double angle3_, double angle4_)
    : observed_x(observed_x_), observed_y(observed_y_),
      marker_x(marker_x_), marker_y(marker_y_), marker_z(marker_z_),
      angle0(angle0_), angle1(angle1_), angle2(angle2_), angle3(angle3_), angle4(angle4_) {}

bool NewProjectionErrorCostFuctor::operator()(const double *const offset_angle0,
                                              const double *const offset_angle1,
                                              const double *const offset_angle2,
                                              const double *const offset_angle3,
                                              const double *const offset_angle4,
                                              double *residuals) const
{
    // PassiveArm passivearm(link);
    PassiveArm passivearm;
    passivearm.q[0] = angle0 + offset_angle0[0];
    passivearm.q[1] = angle1 + offset_angle1[0];
    passivearm.q[2] = angle2 + offset_angle2[0];
    passivearm.q[3] = angle3 + offset_angle3[0];
    passivearm.q[4] = angle4 + offset_angle4[0];
    passivearm.forward_kinematics();

    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;     // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
    Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;

    double Quaternion[4];
    double Transform[3];
    Transform[0] = Ptip[0];
    Transform[1] = Ptip[1];
    Transform[2] = Ptip[2];
    Transform::RotMatToQuaternion(&Quaternion[0], &Quaternion[1], &Quaternion[2], &Quaternion[3],
                                  endoscope_pose[0][0], endoscope_pose[0][1], endoscope_pose[0][2],
                                  endoscope_pose[1][0], endoscope_pose[1][1], endoscope_pose[1][2],
                                  endoscope_pose[2][0], endoscope_pose[2][1], endoscope_pose[2][2]);

    double point[3]; // マーカーの三次元点
    point[0] = marker_x;
    point[1] = marker_y;
    point[2] = marker_z;

    double p[3]; //カメラ座標系でのマーカー位置
    ceres::QuaternionRotatePoint(Quaternion, point, p);

    // カメラ座標系への観測点の座標変換（並進）camera[4,5,6]
    p[0] += Transform[0];
    p[1] += Transform[1];
    p[2] += Transform[2];

    // ｚ座標を1とする
    double xp = p[0] / p[2];
    double yp = p[1] / p[2];

    // 最終投影位置の計算
    const double focal_x = 396.7;
    const double focal_y = 396.9;
    const double u_x = 163.6;
    const double u_y = 157.1;
    double predicted_x = focal_x * xp + u_x;
    double predicted_y = focal_y * yp + u_y;

    /* ここから */
    // 投影計算をopencvのprojectpoints()に任せる
    cv::Mat objectPoints = (cv::Mat_<double>(3, 1) << point[0], point[1], point[2]);

    cv::Mat rvec, rotationMatrix = (cv::Mat_<double>(3, 3) << endoscope_pose[0][0], endoscope_pose[0][1], endoscope_pose[0][2],
                                    endoscope_pose[1][0], endoscope_pose[1][1], endoscope_pose[1][2],
                                    endoscope_pose[2][0], endoscope_pose[2][1], endoscope_pose[2][2]);
    ;
    cv::Rodrigues(rotationMatrix, rvec);

    cv::Mat tvec(3, 1, CV_64FC1);
    tvec.at<double>(0) = Ptip[0];
    tvec.at<double>(1) = Ptip[1];
    tvec.at<double>(2) = Ptip[2];

    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    const double fovx = 396.7, fovy = 396.9, u0 = 163.6, v0 = 157.1;
    cameraMatrix = (cv::Mat_<double>(3, 3) << fovx, 0.0, u0,
                    0.0, fovy, v0,
                    0.0, 0.0, 1.0);

    cv::Mat distcoeffs = (cv::Mat_<double>(5, 1) << 0., 0., 0., 0., 0.);

    cv::Mat imagePoints(2, 1, CV_64FC1);
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distcoeffs, imagePoints);
    double predicted_X = imagePoints.at<double>(0);
    double predicted_Y = imagePoints.at<double>(1);

    /* ここまで */
    // printf("mycalc:[%lf %lf]\n", predicted_x, predicted_y);
    // printf("opencv:[%lf %lf]\n", predicted_X, predicted_Y);
    // printf("opencv:[%lf %lf]\n", imagePoints.at<double>(0), imagePoints.at<double>(1));
    // printf("ptip:[%lf %lf %lf]\n", Ptip[0], Ptip[1], Ptip[2]);
    // printf("tran:[%lf %lf %lf]\n", Transform[0], Transform[1], Transform[2]);
    // printf("obs:[%lf %lf]\n", observed_x, observed_y);
    // residuals[0] = predicted_x - observed_x;
    // residuals[1] = predicted_y - observed_y;
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
}
ceres::CostFunction *NewProjectionErrorCostFuctor::Create(const double observed_x,
                                                          const double observed_y,
                                                          const double marker_x,
                                                          const double marker_y,
                                                          const double marker_z,
                                                          const double angle0,
                                                          const double angle1,
                                                          const double angle2,
                                                          const double angle3,
                                                          const double angle4)
{
    return (new ceres::NumericDiffCostFunction<NewProjectionErrorCostFuctor, ceres::CENTRAL, 2, 1, 1, 1, 1, 1>(
        new NewProjectionErrorCostFuctor(observed_x, observed_y, marker_x, marker_y, marker_z, angle0, angle1, angle2, angle3, angle4)));
}