#ifndef COST_FUNCTION_HPP_
#define COST_FUNCTION_HPP_

#include "ceres/ceres.h"
#include "Scene.hpp"

struct ProjectionErrorCostFuctor
{
    ProjectionErrorCostFuctor(double observed_x_, double observed_y_,
                              double marker_x_, double marker_y_, double marker_z_,
                              Joint *angle);

    bool operator()(const double *const time_delay,
                    double *residuals) const;

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y,
                                       const double marker_x,
                                       const double marker_y,
                                       const double marker_z,
                                       Joint *angle);

private:
    double observed_x;
    double observed_y;
    double marker_x;
    double marker_y;
    double marker_z;
    Joint *angle;

public:
    int counter;
};

#endif