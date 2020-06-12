#include <ktl.h>

#include "../include/arm.hpp"
#include "../include/cost_function_link.hpp"

ProjectionErrorCostFuctor::ProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                                                     double marker_x_, double marker_y_, double marker_z_,
                                                     double angle0_, double angle1_, double angle2_, double angle3_, double angle4_)
    : observed_x(observed_x_), observed_y(observed_y_),
      marker_x(marker_x_), marker_y(marker_y_), marker_z(marker_z_),
      angle0(angle0_), angle1(angle1_), angle2(angle2_), angle3(angle3_), angle4(angle4_) {}

bool ProjectionErrorCostFuctor::operator()(const double *const link0_x,
                                           const double *const link0_y,
                                           const double *const link0_z,
                                           const double *const link1_x,
                                           const double *const link1_y,
                                           const double *const link1_z,
                                           const double *const link2_x,
                                           const double *const link2_y,
                                           const double *const link2_z,
                                           const double *const link3_x,
                                           const double *const link3_y,
                                           const double *const link3_z,
                                           const double *const link4_x,
                                           const double *const link4_y,
                                           const double *const link4_z,
                                           const double *const link5_x,
                                           const double *const link5_y,
                                           const double *const link5_z,
                                           const double *const link6_x,
                                           const double *const link6_y,
                                           const double *const link6_z,
                                           const double *const offset_angle0,
                                           const double *const offset_angle1,
                                           const double *const offset_angle2,
                                           const double *const offset_angle3,
                                           const double *const offset_angle4,
                                           double *residuals) const
{
    PassiveArm passivearm(link0_x, link0_y, link0_z,
                          link1_x, link1_y, link1_z,
                          link2_x, link2_y, link2_z,
                          link3_x, link3_y, link3_z,
                          link4_x, link4_y, link4_z,
                          link5_x, link5_y, link5_z);
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

    //　3つめの透視射影
    double point[3]; // マーカーの三次元点(ワールド座標系)
    point[0] = marker_x;
    point[1] = marker_y;
    point[2] = marker_z;

    double p[2];
    p[0] = endoscope_pose[0][0] * point[0] + endoscope_pose[1][0] * point[1] + endoscope_pose[2][0] * point[2];
    p[1] = endoscope_pose[0][1] * point[0] + endoscope_pose[1][1] * point[1] + endoscope_pose[2][1] * point[2];
    p[2] = endoscope_pose[0][2] * point[0] + endoscope_pose[1][2] * point[1] + endoscope_pose[2][2] * point[2];

    double t[3];
    t[0] = endoscope_pose[0][0] * Ptip[0] + endoscope_pose[1][0] * Ptip[1] + endoscope_pose[2][0] * Ptip[2];
    t[1] = endoscope_pose[0][1] * Ptip[0] + endoscope_pose[1][1] * Ptip[1] + endoscope_pose[2][1] * Ptip[2];
    t[2] = endoscope_pose[0][2] * Ptip[0] + endoscope_pose[1][2] * Ptip[1] + endoscope_pose[2][2] * Ptip[2];

    double xp = (p[0] - t[0]) / (p[2] - t[2]);
    double yp = (p[1] - t[1]) / (p[2] - t[2]);

    // 最終投影位置の計算
    const double focal_x = 396.7;
    const double focal_y = 396.9;
    const double u_x = 163.6;
    const double u_y = 157.1;
    residuals[0] = focal_x * xp + u_x;
    residuals[1] = focal_y * yp + u_y;
    return true;
}
ceres::CostFunction *ProjectionErrorCostFuctor::Create(const double observed_x,
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
    return (new ceres::NumericDiffCostFunction<ProjectionErrorCostFuctor, ceres::CENTRAL, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
        new ProjectionErrorCostFuctor(observed_x, observed_y, marker_x, marker_y, marker_z, angle0, angle1, angle2, angle3, angle4)));
}