#include <ktl.h>

#include "../include/calibration_delay/arm.hpp"
#include "../include/calibration_delay/cost_function.hpp"

ProjectionErrorCostFuctor::ProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                                                     double marker_x_, double marker_y_, double marker_z_,
                                                     Joint *angle_)
    : observed_x(observed_x_), observed_y(observed_y_),
      marker_x(marker_x_), marker_y(marker_y_), marker_z(marker_z_),
      angle(angle_),
      counter(0) {}

bool ProjectionErrorCostFuctor::operator()(const double *const time_delay,
                                           double *residuals) const
{
    int num = (int)(time_delay[0] * 1000.);
    printf("num = %d\n", num);

    PassiveArm passivearm;
    passivearm.q[0] = angle[num].theta_0;
    passivearm.q[1] = angle[num].theta_1;
    passivearm.q[2] = angle[num].theta_2;
    passivearm.q[3] = angle[num].theta_3;
    passivearm.q[4] = angle[num].theta_4;

    passivearm.forward_kinematics();

    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG); //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;     // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);                     // 内視鏡の向き
    Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;

    //　3つめの透視射影
    double point[3]; // マーカーの三次元点(ワールド座標系)
    point[0] = marker_x;
    point[1] = marker_y;
    point[2] = marker_z;

    double Point[3];
    for (int i = 0; i < 3; i++)
    {
        Point[i] = endoscope_pose[0][i] * (point[0] - Ptip[0]) + endoscope_pose[1][i] * (point[1] - Ptip[1]) + endoscope_pose[2][i] * (point[2] - Ptip[2]);
    }

    double xp = Point[0] / Point[2];
    double yp = Point[1] / Point[2];

    // 最終投影位置の計算
    const double focal_x = 396.7;
    const double focal_y = 396.9;
    const double u_x = 163.6;
    const double u_y = 157.1;
    double predicted_x = focal_x * xp + u_x;
    double predicted_y = focal_y * yp + u_y;

    double error[2];
    error[0] = predicted_x - observed_x;
    error[1] = predicted_y - observed_y;
    double error_distance = sqrt(error[0] * error[0] + error[1] * error[1]);

    if (error_distance > 100.)
    {
        // デカすぎで逆におかしいので誤差に追加しない
        residuals[0] = 0.0;
        residuals[1] = 0.0;
    }
    else
    {
        residuals[0] = error[0];
        residuals[1] = error[1];
    }

    return true;
}
ceres::CostFunction *ProjectionErrorCostFuctor::Create(const double observed_x,
                                                       const double observed_y,
                                                       const double marker_x,
                                                       const double marker_y,
                                                       const double marker_z,
                                                       Joint *angle)
{
    return (new ceres::NumericDiffCostFunction<ProjectionErrorCostFuctor, ceres::CENTRAL, 2, 1>(
        new ProjectionErrorCostFuctor(observed_x, observed_y, marker_x, marker_y, marker_z, angle)));
}