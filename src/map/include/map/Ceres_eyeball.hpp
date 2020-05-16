#ifndef CERES_EYEBALL_HPP__
#define CERES_EYEBALL_HPP__

#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// コスト関数構造体
struct EllipseResiduals
{
    // データを一つずつ入力する際に、このコンストラクタが呼ばれる
    EllipseResiduals(double x, double y, double z)
        : x_(x), y_(y), z_(z) {}

    // コスト関数の定義
    template <typename T>
    bool operator()(const T *const ellipse,
                    const T *const center,
                    T *residual) const
    {
        residual[0] = ((T(x_) - center[0]) * (T(x_) - center[0])) / (ellipse[0] * ellipse[0]) +
                      ((T(y_) - center[1]) * (T(y_) - center[1])) / (ellipse[1] * ellipse[1]) +
                      ((T(z_) - center[2]) * (T(z_) - center[2])) / (ellipse[2] * ellipse[2]) - 1.;
        return true;
    }

    static ceres::CostFunction *Create(const double x_,
                                       const double y_,
                                       const double z_)
    {
        return (new ceres::AutoDiffCostFunction<EllipseResiduals, 1, 3, 3>(new EllipseResiduals(x_, y_, z_)));
    }

private:
    // 1組のデータ
    const double x_;
    const double y_;
    const double z_;
};
#endif