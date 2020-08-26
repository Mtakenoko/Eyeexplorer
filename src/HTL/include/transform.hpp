#ifndef ARM_STATUS__TRANSFORM_
#define ARM_STATUS__TRANSFORM_

#include <opencv2/opencv.hpp>

class Transform
{
public:
    explicit Transform();
    
    static bool RotMatToQuaternion(
        double *qx, double *qy, double *qz, double *qw,
        const double &m11, const double &m12, const double &m13,
        const double &m21, const double &m22, const double &m23,
        const double &m31, const double &m32, const double &m33);

    bool RotMatToQuaternion(
        float *qx, float *qy, float *qz, float *qw,
        const float &m11, const float &m12, const float &m13,
        const float &m21, const float &m22, const float &m23,
        const float &m31, const float &m32, const float &m33);


    void QuaternionToEulerAngles(
        double q0, double q1, double q2, double q3,
        double &roll, double &pitch, double &yaw);
    void QuaternionToRotMat(
        float &m11, float &m12, float &m13,
        float &m21, float &m22, float &m23,
        float &m31, float &m32, float &m33,
        float qx, float qy, float qz, float qw);
    cv::Mat QuaternionToRotMat(const float &qx, const float &qy, const float &qz, const float &qw);
    float RevFromRotMat(cv::Mat R);
    void RotMatToAngles(cv::Mat R,
                        double &angle_x, double &angle_y, double &angle_z);
    void CVRottoDouble(cv::Mat inputmatrix, double &output);

private:
};
#endif