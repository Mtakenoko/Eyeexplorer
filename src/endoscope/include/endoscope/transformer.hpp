#ifndef _TRANSFORM_
#define _TRANSFORM_

#include <opencv2/opencv.hpp>

template <class T>
class Transformer
{
public:
    static bool RotMatToQuaternion(
        T *qx, T *qy, T *qz, T *qw,
        const T &m11, const T &m12, const T &m13,
        const T &m21, const T &m22, const T &m23,
        const T &m31, const T &m32, const T &m33);

    static void QuaternionToEulerAngles(
        T q0, T q1, T q2, T q3,
        const T &roll, const T &pitch, const T &yaw);
    static void QuaternionToRotMat(
        T &m11, T &m12, T &m13,
        T &m21, T &m22, T &m23,
        T &m31, T &m32, T &m33,
        const T &qx, const T &qy, const T &qz, const T &qw);
    static cv::Mat QuaternionToRotMat(const T &qx, const T &qy, const T &qz, const T &qw);
    static T RevFromRotMat(cv::Mat R);
    static void RotMatToAngles(cv::Mat R,
                               T &angle_x, T &angle_y, T &angle_z);
};

template <class T>
bool Transformer<T>::RotMatToQuaternion(
    T *qx, T *qy, T *qz, T *qw,
    const T &m11, const T &m12, const T &m13,
    const T &m21, const T &m22, const T &m23,
    const T &m31, const T &m32, const T &m33)

{
    // 最大成分を検索
    T elem[4]; // 0:x, 1:y, 2:z, 3:w
    elem[0] = m11 - m22 - m33 + 1.0;
    elem[1] = -m11 + m22 - m33 + 1.0;
    elem[2] = -m11 - m22 + m33 + 1.0;
    elem[3] = m11 + m22 + m33 + 1.0;

    unsigned biggestIndex = 0;
    for (int i = 1; i < 4; i++)
    {
        if (elem[i] > elem[biggestIndex])
            biggestIndex = i;
    }

    if (elem[biggestIndex] < 0.0f)
        return false; // 引数の行列に間違いあり！

    // 最大要素の値を算出
    T *q[4] = {qx, qy, qz, qw};
    T v = sqrtf(elem[biggestIndex]) * 0.5f;
    *q[biggestIndex] = v;
    T mult = 0.25f / v;

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

template <class T>
void Transformer<T>::QuaternionToEulerAngles(T q0, T q1, T q2, T q3,
                                           const T &roll, const T &pitch, const T &yaw)
{
    T q0q0 = q0 * q0;
    T q0q1 = q0 * q1;
    T q0q2 = q0 * q2;
    T q0q3 = q0 * q3;
    T q1q1 = q1 * q1;
    T q1q2 = q1 * q2;
    T q1q3 = q1 * q3;
    T q2q2 = q2 * q2;
    T q2q3 = q2 * q3;
    T q3q3 = q3 * q3;
    roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
    pitch = asin(2.0 * (q0q2 - q1q3));
    yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

template <class T>
void Transformer<T>::QuaternionToRotMat(
    T &m11, T &m12, T &m13,
    T &m21, T &m22, T &m23,
    T &m31, T &m32, T &m33,
    const T &qx, const T &qy, const T &qz, const T &qw)
{
    m11 = 1.0f - 2.0 * qy * qy - 2.0 * qz * qz;
    m12 = 2.0 * qx * qy + 2.0 * qw * qz;
    m13 = 2.0 * qx * qz - 2.0 * qw * qy;

    m21 = 2.0 * qx * qy - 2.0 * qw * qz;
    m22 = 1.0f - 2.0 * qx * qx - 2.0 * qz * qz;
    m23 = 2.0 * qy * qz + 2.0 * qw * qx;

    m31 = 2.0 * qx * qz + 2.0 * qw * qy;
    m32 = 2.0 * qy * qz - 2.0 * qw * qx;
    m33 = 1.0f - 2.0 * qx * qx - 2.0 * qy * qy;
}

template <class T>
cv::Mat Transformer<T>::QuaternionToRotMat(const T &qx, const T &qy, const T &qz, const T &qw)
{
    cv::Mat Output(3, 3, CV_32FC1);
    Output.at<T>(0, 0) = qx * qx - qy * qy - qz * qz + qw * qw;
    Output.at<T>(0, 1) = 2.0 * (qx * qy - qz * qw);
    Output.at<T>(0, 2) = 2.0 * (qx * qz + qy * qw);
    Output.at<T>(1, 0) = 2.0 * (qx * qy + qz * qw);
    Output.at<T>(1, 1) = -qx * qx + qy * qy - qz * qz + qw * qw;
    Output.at<T>(1, 2) = 2.0 * (qy * qz - qx * qw);
    Output.at<T>(2, 0) = 2.0 * (qx * qz - qy * qw);
    Output.at<T>(2, 1) = 2.0 * (qy * qz + qx * qw);
    Output.at<T>(2, 2) = -qx * qx - qy * qy + qz * qz + qw * qw;
    return Output;
}

template <class T>
T Transformer<T>::RevFromRotMat(cv::Mat R_arm)
{
    //回転行列をクォータニオンに変換
    T qx, qy, qz, qw;
    Transformer::RotMatToQuaternion(&qx, &qy, &qz, &qw,
                                  R_arm.at<T>(0, 0), R_arm.at<T>(0, 1), R_arm.at<T>(0, 2),
                                  R_arm.at<T>(1, 0), R_arm.at<T>(1, 1), R_arm.at<T>(1, 2),
                                  R_arm.at<T>(2, 0), R_arm.at<T>(2, 1), R_arm.at<T>(2, 2));
    //クォータニオンの4つめの要素から回転角を取り出す
    T phi = 2 * std::acos(qw);
    return phi;
}

template <class T>
void Transformer<T>::RotMatToAngles(cv::Mat R,
                                  T &angle_x, T &angle_y, T &angle_z)
{
    T threshhold = 0.001;

    if (abs(R.at<T>(2, 1) - 1.0) < threshhold)
    {
        angle_x = M_PI / 2;
        angle_y = 0;
        angle_z = atan2(R.at<T>(1, 0), R.at<T>(0, 0));
    }
    else if (abs(R.at<T>(2, 1) + 1.0) < threshhold)
    {
        angle_x = -M_PI / 2;
        angle_y = 0;
        angle_z = atan2(R.at<T>(1, 0), R.at<T>(0, 0));
    }
    else
    {
        angle_x = asin(R.at<T>(2, 1));
        angle_y = atan2(R.at<T>(2, 0), R.at<T>(2, 2));
        angle_z = atan2(R.at<T>(0, 1), R.at<T>(1, 1));
    }
}
#endif