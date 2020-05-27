#ifndef CERES_PARAM2_HPP__
#define CERES_PARAM2_HPP__

#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <ktl.h>
#include "arm.hpp"
#include "../../../HTL/include/transform.h"

struct NewProjectionErrorCostFuctor
{
    NewProjectionErrorCostFuctor(double observed_x_, double observed_y_, double marker_x_, double marker_y_, double marker_z_)
        : observed_x(observed_x_), observed_y(observed_y_),
          marker_x(marker_x_), marker_y(marker_y_), marker_z(marker_z_) {}

    bool operator()(const double *const link,
                    const double *const angle,
                    double *residuals) const
    {
        PassiveArm passivearm(link);
        for (int i = 0; i < 5; i++)
        {
            passivearm.q[i] = angle[i];
        }
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

        double p[3];  //カメラ座標系でのマーカー位置
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

        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y,
                                       const double marker_x,
                                       const double marker_y,
                                       const double marker_z)
    {
        return (new ceres::NumericDiffCostFunction<NewProjectionErrorCostFuctor, ceres::CENTRAL, 2, 21, 3>(
            new NewProjectionErrorCostFuctor(observed_x, observed_y, marker_x, marker_y, marker_z)));
    }

private:
    double observed_x;
    double observed_y;
    double marker_x;
    double marker_y;
    double marker_z;
};

#endif