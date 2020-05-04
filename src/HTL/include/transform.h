#ifndef ARM_STATUS__TRANSFORM_
#define ARM_STATUS__TRANSFORM_

#include <opencv2/opencv.hpp>

class Transform
{
public:
    explicit Transform();
    bool RotMatToQuaternion(
        float &qx, float &qy, float &qz, float &qw,
        float m11, float m12, float m13,
        float m21, float m22, float m23,
        float m31, float m32, float m33);
    void QuaternionToEulerAngles(
        double q0, double q1, double q2, double q3,
        double &roll, double &pitch, double &yaw);
    void QuaternionToRotMat(
        float &m11, float &m12, float &m13,
        float &m21, float &m22, float &m23,
        float &m31, float &m32, float &m33,
        float qx, float qy, float qz, float qw);
    cv::Mat QuaternionToRotMat2(const float &qx, const float &qy, const float &qz, const float &qw);
    float RevFromRotMat(cv::Mat R);
    void RotMatToAngles(cv::Mat R,
                        double &angle_x, double &angle_y, double &angle_z);
    void CVRottoDouble(cv::Mat inputmatrix, double &output);

private:
};
#endif