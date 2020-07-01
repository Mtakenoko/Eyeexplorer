#ifndef COST_FUNCTION_HPP_
#define COST_FUNCTION_HPP_

#include "ceres/ceres.h"

struct ProjectionErrorCostFuctor
{
    ProjectionErrorCostFuctor(double observed_x_, double observed_y_);

    bool operator()(const double *const camera_angle_0,
                    const double *const camera_angle_1,
                    const double *const camera_angle_2,
                    const double *const camera_pos_x,
                    const double *const camera_pos_y,
                    const double *const camera_pos_z,
                    const double *const point_x,
                    const double *const point_y,
                    const double *const point_z,
                    double *residuals) const;

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y);

private:
    double observed_x;
    double observed_y;
};

#endif