#include <cstdio>
#include <iostream>
#include <string>
#include <ktl.h>

#include "../include/transform.h"

Transform::Transform()
{
}

bool Transform::RotMatToQuaternion(
    double *qx, double *qy, double *qz, double *qw,
    const double &m11, const double &m12, const double &m13,
    const double &m21, const double &m22, const double &m23,
    const double &m31, const double &m32, const double &m33)
{
    // 最大成分を検索
    double elem[4]; // 0:x, 1:y, 2:z, 3:w
    elem[0] = m11 - m22 - m33 + 1.0f;
    elem[1] = -m11 + m22 - m33 + 1.0f;
    elem[2] = -m11 - m22 + m33 + 1.0f;
    elem[3] = m11 + m22 + m33 + 1.0f;

    unsigned biggestIndex = 0;
    for (int i = 1; i < 4; i++)
    {
        if (elem[i] > elem[biggestIndex])
            biggestIndex = i;
    }

    if (elem[biggestIndex] < 0.0f)
        return false; // 引数の行列に間違いあり！

    // 最大要素の値を算出
    double *q[4] = {qx, qy, qz, qw};
    double v = sqrtf(elem[biggestIndex]) * 0.5f;
    *q[biggestIndex] = v;
    double mult = 0.25f / v;

    switch (biggestIndex)
    {
    case 0: // x
        *q[1] = (m12 + m21) * mult;
        *q[2] = (m31 + m13) * mult;
        *q[3] = (m32 - m23) * mult;
        break;
    case 1: // y
        *q[0] = (m12 + m21) * mult;
        *q[2] = (m23 + m32) * mult;
        *q[3] = (m13 - m31) * mult;
        break;
    case 2: // z
        *q[0] = (m31 + m13) * mult;
        *q[1] = (m23 + m32) * mult;
        *q[3] = (m21 - m12) * mult;
        break;
    case 3: // w
        *q[0] = (m32 - m23) * mult;
        *q[1] = (m13 - m31) * mult;
        *q[2] = (m21 - m12) * mult;
        break;
    }

    return true;
}
bool Transform::RotMatToQuaternion(
    float *qx, float *qy, float *qz, float *qw,
    const float &m11, const float &m12, const float &m13,
    const float &m21, const float &m22, const float &m23,
    const float &m31, const float &m32, const float &m33)
{
    // 最大成分を検索
    float elem[4]; // 0:x, 1:y, 2:z, 3:w
    elem[0] = m11 - m22 - m33 + 1.0f;
    elem[1] = -m11 + m22 - m33 + 1.0f;
    elem[2] = -m11 - m22 + m33 + 1.0f;
    elem[3] = m11 + m22 + m33 + 1.0f;

    unsigned biggestIndex = 0;
    for (int i = 1; i < 4; i++)
    {
        if (elem[i] > elem[biggestIndex])
            biggestIndex = i;
    }

    if (elem[biggestIndex] < 0.0f)
        return false; // 引数の行列に間違いあり！

    // 最大要素の値を算出
    float *q[4] = {qx, qy, qz, qw};
    float v = sqrtf(elem[biggestIndex]) * 0.5f;
    *q[biggestIndex] = v;
    float mult = 0.25f / v;

    switch (biggestIndex)
    {
    case 0: // x
        *q[1] = (m12 + m21) * mult;
        *q[2] = (m31 + m13) * mult;
        *q[3] = (m32 - m23) * mult;
        break;
    case 1: // y
        *q[0] = (m12 + m21) * mult;
        *q[2] = (m23 + m32) * mult;
        *q[3] = (m13 - m31) * mult;
        break;
    case 2: // z
        *q[0] = (m31 + m13) * mult;
        *q[1] = (m23 + m32) * mult;
        *q[3] = (m21 - m12) * mult;
        break;
    case 3: // w
        *q[0] = (m32 - m23) * mult;
        *q[1] = (m13 - m31) * mult;
        *q[2] = (m21 - m12) * mult;
        break;
    }

    return true;
}

void Transform::QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                                        double &roll, double &pitch, double &yaw)
{
    double q0q0 = q0 * q0;
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    double q0q3 = q0 * q3;
    double q1q1 = q1 * q1;
    double q1q2 = q1 * q2;
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

void Transform::QuaternionToRotMat(
    float &m11, float &m12, float &m13,
    float &m21, float &m22, float &m23,
    float &m31, float &m32, float &m33,
    float qx, float qy, float qz, float qw)
{
    m11 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    m12 = 2.0f * qx * qy + 2.0f * qw * qz;
    m13 = 2.0f * qx * qz - 2.0f * qw * qy;

    m21 = 2.0f * qx * qy - 2.0f * qw * qz;
    m22 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    m23 = 2.0f * qy * qz + 2.0f * qw * qx;

    m31 = 2.0f * qx * qz + 2.0f * qw * qy;
    m32 = 2.0f * qy * qz - 2.0f * qw * qx;
    m33 = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
}
cv::Mat Transform::QuaternionToRotMat(const float &qx, const float &qy, const float &qz, const float &qw)
{
    cv::Mat Output(3, 3, CV_32FC1);
    Output.at<float>(0, 0) = qx * qx - qy * qy - qz * qz + qw * qw;
    Output.at<float>(0, 1) = 2.0f * (qx * qy - qz * qw);
    Output.at<float>(0, 2) = 2.0f * (qx * qz + qy * qw);
    Output.at<float>(1, 0) = 2.0f * (qx * qy + qz * qw);
    Output.at<float>(1, 1) = -qx * qx + qy * qy - qz * qz + qw * qw;
    Output.at<float>(1, 2) = 2.0f * (qy * qz - qx * qw);
    Output.at<float>(2, 0) = 2.0f * (qx * qz - qy * qw);
    Output.at<float>(2, 1) = 2.0f * (qy * qz + qx * qw);
    Output.at<float>(2, 2) = -qx * qx - qy * qy + qz * qz + qw * qw;
    return Output;
}

float Transform::RevFromRotMat(cv::Mat R_arm)
{
    //回転行列をクォータニオンに変換
    float qx, qy, qz, qw;
    Transform::RotMatToQuaternion(&qx, &qy, &qz, &qw,
                                  R_arm.at<float>(0, 0), R_arm.at<float>(0, 1), R_arm.at<float>(0, 2),
                                  R_arm.at<float>(1, 0), R_arm.at<float>(1, 1), R_arm.at<float>(1, 2),
                                  R_arm.at<float>(2, 0), R_arm.at<float>(2, 1), R_arm.at<float>(2, 2));
    //クォータニオンの4つめの要素から回転角を取り出す
    float phi = 2 * std::acos(qw);
    return phi;
}

void Transform::RotMatToAngles(cv::Mat R, double &angle_x, double &angle_y, double &angle_z)
{
    double threshhold = 0.001;

    if (abs(R.at<double>(2, 1) - 1.0) < threshhold)
    {
        angle_x = PI / 2;
        angle_y = 0;
        angle_z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else if (abs(R.at<double>(2, 1) + 1.0) < threshhold)
    {
        angle_x = -PI / 2;
        angle_y = 0;
        angle_z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        angle_x = asin(R.at<double>(2, 1));
        angle_y = atan2(R.at<double>(2, 0), R.at<double>(2, 2));
        angle_z = atan2(R.at<double>(0, 1), R.at<double>(1, 1));
    }
}