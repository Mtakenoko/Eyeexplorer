#include <cstdio>
#include <iostream>
#include <string>
#include <ktl.h>

#include "../include/arm/transform.h"

Transform::Transform()
{
}

bool Transform::RotMatToQuaternion(
    float &qx, float &qy, float &qz, float &qw,
    float m11, float m12, float m13,
    float m21, float m22, float m23,
    float m31, float m32, float m33)
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
    float *q[4] = {&qx, &qy, &qz, &qw};
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