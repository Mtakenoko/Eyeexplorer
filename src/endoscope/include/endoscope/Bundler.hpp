#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// Templated pinhole camera model for used with Ceres.
// The camera is parameterized using 12 parameters: 3 for rotation, 3 for translation(x, y, z), 2 for
// focal length, 2 for c_x,c_y and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
// 再投影誤差のコスト関数構造体
struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}
    template <typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const
    {
        // cameraはワールド座標系でのカメラ位置姿勢
        // pointはワールド座標系での観測点の位置
        // pはカメラ座標系での観測点の位置
        // T p_[3];
        T p[3];

        // 観測点の座標変換（並進）camera[3,4,5]
        // p_[0] = point[0] - camera[3];
        // p_[1] = point[1] - camera[4];
        // p_[2] = point[2] - camera[5];

        // 観測点の座標変換（回転）camera[0,1,2]
        ceres::AngleAxisRotatePoint(camera, point, p);

        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // ｚ座標を1とする
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];

        // 最終投影位置の計算
        // const T focal_x = 396.7;
        // const T focal_y = 396.9;
        // const T u_x = 163.6;
        // const T u_y = 157.1;

        T predicted_x = 396.7 * xp + 163.6;
        T predicted_y = 396.9 * yp + 157.1;

        // // 観測点との誤差の計算
        // static int count = 0;
        // if (count % 10 == 0)
        // {
        //     std::cout << "predicted = " << predicted_x << ", " << predicted_y << std::endl
        //               << "observed  = " << observed_x << ", " << observed_y << std::endl;
        // }
        // count++;
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3>(new SnavelyReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};