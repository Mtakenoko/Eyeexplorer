#include "../include/endoscope/cost_function.hpp"
#include <opencv2/opencv.hpp>

ProjectionErrorCostFuctor::ProjectionErrorCostFuctor(double observed_x_, double observed_y_)
    : observed_x(observed_x_), observed_y(observed_y_) {}

bool ProjectionErrorCostFuctor::operator()(const double *const camera_angle_0,
                                           const double *const camera_angle_1,
                                           const double *const camera_angle_2,
                                           const double *const camera_pos_x,
                                           const double *const camera_pos_y,
                                           const double *const camera_pos_z,
                                           const double *const point_x,
                                           const double *const point_y,
                                           const double *const point_z,
                                           double *residuals) const
{
    //　3つめの透視射影
    double point[3]; // マーカーの三次元点(ワールド座標系)
    point[0] = point_x[0];
    point[1] = point_y[0];
    point[2] = point_z[0];

    double Point[3];
    cv::Matx33d pose;
    cv::Matx31d pose_rod;
    pose_rod(0) = camera_angle_0[0];
    pose_rod(1) = camera_angle_1[0];
    pose_rod(2) = camera_angle_2[0];
    cv::Rodrigues(pose_rod, pose);

    for (int i = 0; i < 3; i++)
    {
        Point[i] = pose(0, i) * (point[0] - camera_pos_x[0]) +
                   pose(1, i) * (point[1] - camera_pos_y[0]) +
                   pose(2, i) * (point[2] - camera_pos_z[0]);
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
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    // std::cout << "predicted: " << predicted_x << ", " << predicted_y << std::endl;
    // std::cout << "observed : " << observed_x << ", " << observed_y << std::endl;

    return true;
}
ceres::CostFunction *ProjectionErrorCostFuctor::Create(const double observed_x,
                                                       const double observed_y)
{
    return (new ceres::NumericDiffCostFunction<ProjectionErrorCostFuctor, ceres::CENTRAL, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
        new ProjectionErrorCostFuctor(observed_x, observed_y)));
}