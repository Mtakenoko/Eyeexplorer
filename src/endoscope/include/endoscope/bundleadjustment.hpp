#include <cmath>
#include <cstdio>
#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
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
        T p[3];

        // 観測点の座標変換（回転）camera[0,1,2]
        ceres::QuaternionRotatePoint(camera, point, p);
        // 観測点の座標変換（並進）camera[4,5,6]
        p[0] += camera[4];
        p[1] += camera[5];
        p[2] += camera[6];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        // ｚ座標の補正
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion.
        //　歪み補正
        const T &l1 = camera[8];
        const T &l2 = camera[9];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        // 最終投影位置の計算
        const T &focal = camera[7];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        // 観測点との誤差の計算
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;
        
        // //ここから
        //  // cameraはワールド座標系でのカメラ位置姿勢
        // // pointはワールド座標系での観測点の位置
        // // pはカメラ座標系での観測点の位置
        // T p[3];

        // // 観測点の座標変換（回転）camera[0,1,2]
        // ceres::QuaternionRotatePoint(camera, point, p);
        // // 観測点の座標変換（並進）camera[4,5,6]
        // p[0] += camera[4];
        // p[1] += camera[5];
        // p[2] += camera[6];

        return true;
    }
    // Factory to hide the construction of the CostFunction object from
    // the client code.
    // 構造体の中にコスト関数を生成するコードを隠蔽する
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 10, 3>(new SnavelyReprojectionError(observed_x, observed_y)));
    }
    double observed_x;
    double observed_y;
};