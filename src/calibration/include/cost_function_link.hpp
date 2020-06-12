#ifndef COST_FUNCTION_LINK_HPP_
#define COST_FUNCTION_LINK_HPP_

#include "ceres/ceres.h"

struct ProjectionErrorCostFuctor
{
    ProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                              double marker_x_, double marker_y_, double marker_z_,
                              double angle0_, double angle1_, double angle2_, double angle3_, double angle4_);

    bool operator()(const double *const link0_x,
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
                    double *residuals) const;

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y,
                                       const double marker_x,
                                       const double marker_y,
                                       const double marker_z,
                                       const double angle0,
                                       const double angle1,
                                       const double angle2,
                                       const double angle3,
                                       const double angle4);

private:
    double observed_x;
    double observed_y;
    double marker_x;
    double marker_y;
    double marker_z;
    double angle0;
    double angle1;
    double angle2;
    double angle3;
    double angle4;
};

#endif