#ifndef CERES_PARAM_HPP_
#define CERES_PARAM_HPP_

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
struct ProjectionError
{
    ProjectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T *const link,
                    const T *const angle,
                    T *residuals) const
    {
        // ワールド座標系でのマーカーの位置
        T point[3];
        point[0] = T(100.);
        point[1] = T(100.);
        point[2] = T(100.);

        // ceres::QuaternionProduct();
        // ceres::ConstMatrixRef();
        // ceres::Matrix

        // 順運動学
        T Rotation[4]; // クォータニオン表記
        T E[4];
        E[0] = T(0.);
        E[1] = T(0.);
        E[2] = T(0.);
        E[3] = T(1.);
        T R1[4], R2[4], R3[4], R4[4], R5[4];
        quaternion(0, 0, 1, angle[0], R1);
        quaternion(0, 1, 0, angle[1], R2);
        quaternion(0, 1, 0, angle[2], R3);
        quaternion(1, 0, 0, angle[3], R4);
        quaternion(0, 1, 0, angle[4], R5);
        ceres::QuaternionProduct(E, R1, Rotation);
        ceres::QuaternionProduct(R1, R2, Rotation);
        ceres::QuaternionProduct(R2, R3, Rotation);
        ceres::QuaternionProduct(R3, R4, Rotation);
        ceres::QuaternionProduct(R4, R5, Rotation);

        T Transform[3];
        T link1[3], link2[3], link3[3], link4[3], link5[3], link6[3], link7[3];
        createVec(link[0], link[1], link[2], link1);
        createVec(link[3], link[4], link[5], link2);
        createVec(link[6], link[7], link[8], link3);
        createVec(link[9], link[10], link[11], link4);
        createVec(link[12], link[13], link[14], link5);
        createVec(link[15], link[16], link[17], link6);
        createVec(link[18], link[19], link[20], link7);
        createVec(T(0.), T(0.), T(0.), Transform);

        plusquat(Transform, link7, Transform);
        ceres::QuaternionRotatePoint(R5, Transform, Transform);
        plusquat(Transform, link6, Transform);
        ceres::QuaternionRotatePoint(R4, Transform, Transform);
        plusquat(Transform, link5, Transform);
        ceres::QuaternionRotatePoint(R3, Transform, Transform);
        plusquat(Transform, link4, Transform);
        plusquat(Transform, link3, Transform);
        ceres::QuaternionRotatePoint(R2, Transform, Transform);
        plusquat(Transform, link2, Transform);
        ceres::QuaternionRotatePoint(R1, Transform, Transform);
        plusquat(Transform, link1, Transform);

        // カメラ座標系への観測点の座標変換（回転）camera[0,1,2,3]
        T p[3];
        ceres::QuaternionRotatePoint(Rotation, point, p);

        // カメラ座標系への観測点の座標変換（並進）camera[4,5,6]
        p[0] += Transform[0];
        p[1] += Transform[1];
        p[2] += Transform[2];

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
        return (new ceres::AutoDiffCostFunction<ProjectionError, 2, 21, 3>(
            new ProjectionError(observed_x, observed_y)));
    }
    template <typename T>
    inline void quaternion(const int &x, const int &y, const int &z, const T &theta, T output[4]) const
    {
        output[0] = T(x) * sin(theta);
        output[1] = T(y) * sin(theta);
        output[2] = T(z) * sin(theta);
        output[3] = cos(theta);
    }
    /*
    template <typename T>
    T multiquat(const T &inputquat, const T &rotquat)
    {
        T quat[4];
        quat[0] = rotquat[3] * inputquat[0] - rotquat[2] * inputquat[1] + rotquat[1] * inputquat[2] + rotquat[0] * inputquat[3];
        quat[1] = rotquat[2] * inputquat[0] + rotquat[3] * inputquat[1] - rotquat[0] * inputquat[2] + rotquat[1] * inputquat[3];
        quat[2] = -rotquat[1] * inputquat[0] + rotquat[0] * inputquat[1] + rotquat[3] * inputquat[2] + rotquat[2] * inputquat[3];
        quat[3] = -rotquat[0] * inputquat[0] - rotquat[1] * inputquat[1] - rotquat[2] * inputquat[2] + rotquat[3] * inputquat[3];
        return quat;
    }

    template <typename T>
    T multivecquat(const T &inputquat, const T &rotquat)
    {
        T rotquat_[4];
        rotquat_[0] = -rotquat[0];
        rotquat_[1] = -rotquat[1];
        rotquat_[2] = -rotquat[2];
        rotquat_[3] = rotquat[3];

        T quat[4];
        quat = multiquat(inputquat, rotquat);
        quat = multiquat(rotquat_, quat);

        return quat;
    }
    */
    template <typename T>
    inline void createVec(const T &x, const T &y, const T &z, T output[3]) const
    {
        output[0] = x;
        output[1] = y;
        output[2] = z;
    }

    template <typename T>
    inline void plusquat(const T a[3], const T b[3], T output[3]) const
    {
        output[0] = a[0] + b[0];
        output[1] = a[1] + b[1];
        output[2] = a[2] + b[2];
    }

    double observed_x;
    double observed_y;
};
#endif