#include <ktl.h>

#include "../include/arm.hpp"
#include "../include/cost_function.hpp"

NewProjectionErrorCostFuctor::NewProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                                                           double marker_x_, double marker_y_, double marker_z_,
                                                           double angle0_, double angle1_, double angle2_, double angle3_, double angle4_)
    : observed_x(observed_x_), observed_y(observed_y_),
      marker_x(marker_x_), marker_y(marker_y_), marker_z(marker_z_),
      angle0(angle0_), angle1(angle1_), angle2(angle2_), angle3(angle3_), angle4(angle4_),
      counter(0) {}

bool NewProjectionErrorCostFuctor::operator()(const double *const offset_angle0,
                                              const double *const offset_angle1,
                                              const double *const offset_angle2,
                                              const double *const offset_angle3,
                                              const double *const offset_angle4,
                                              const double *const deflection0,
                                              const double *const deflection1,
                                              double *residuals) const
{
    PassiveArm passivearm;
    passivearm.q[0] = angle0 + offset_angle0[0];
    passivearm.q[1] = angle1 + offset_angle1[0];
    passivearm.q[2] = angle2 + offset_angle2[0];
    passivearm.q[3] = angle3 + offset_angle3[0];
    passivearm.q[4] = angle4 + offset_angle4[0];
    passivearm.forward_kinematics();
    passivearm.calcDeflection(deflection0[0], deflection1[0]);

    //　3つめの透視射影
    double point[3]; // マーカーの三次元点(ワールド座標系)
    point[0] = marker_x;
    point[1] = marker_y;
    point[2] = marker_z;

    double Point[3];
    for (int i = 0; i < 3; i++)
    {
        Point[i] = passivearm.endoscope_pose[0][i] * (point[0] - passivearm.Ptip[0]) +
                   passivearm.endoscope_pose[1][i] * (point[1] - passivearm.Ptip[1]) +
                   passivearm.endoscope_pose[2][i] * (point[2] - passivearm.Ptip[2]);
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
    return (new ceres::NumericDiffCostFunction<NewProjectionErrorCostFuctor, ceres::CENTRAL, 2, 1, 1, 1, 1, 1, 1, 1>(
        new NewProjectionErrorCostFuctor(observed_x, observed_y, marker_x, marker_y, marker_z, angle0, angle1, angle2, angle3, angle4)));
}