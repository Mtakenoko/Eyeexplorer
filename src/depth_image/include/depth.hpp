#ifndef DEPTH_MODEL_HPP_
#define DEPTH_MODEL_HPP_

#include <opencv2/opencv.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <omp.h>

template <class T>
class DepthModel
{
public:
    void setImageInfo(const int &Width, const int &Height, const cv::Mat &CameraMatrix)
    {
        this->width = Width;
        this->height = Height;
        if (CameraMatrix.rows != 3 || CameraMatrix.cols != 3)
        {
            std::cout << "Incorrect cameramatrix " << std::endl;
            return;
        }
        this->cameraMatrix = CameraMatrix;
    }

    void setCameraPose(const cv::Mat &Rot, const cv::Mat trans)
    {
        if (Rot.empty() || trans.empty())
        {
            std::cout << "CameraPose is enpty " << std::endl;
            return;
        }
        this->Rotation = Rot.clone();
        this->transform = trans.clone();
    }

    void setModel(const visualization_msgs::msg::Marker &Model)
    {
        this->model = Model;
    }

    void setMaxDistance(const T &distance)
    {
        this->max_distance = distance;
    }

    cv::Mat create();

private:
    void setProjectionMatrix()
    {
        cv::Mat CamPose;
        cv::hconcat(Rotation.t(), -Rotation.t() * transform, CamPose);
        if (CamPose.rows != 3 || CamPose.cols != 4)
        {
            std::cout << "Incorrect CamPose " << std::endl;
            return;
        }
        this->projectionMatrix = this->cameraMatrix * CamPose;
        if (projectionMatrix.rows != 3 || projectionMatrix.cols != 4)
        {
            std::cout << "Incorrect projectionMatrix " << std::endl;
            return;
        }
    }
    void calcPixelVector(const int &im_x, const int &im_y, cv::Point3_<T> &P, cv::Point3_<T> &v);
    int calcIntersection(const cv::Point3_<T> &P, const cv::Point3_<T> &v, cv::Point3_<T> &point);
    int width;
    int height;
    T max_distance;
    cv::Mat cameraMatrix;
    cv::Mat Rotation;
    cv::Mat transform;
    cv::Mat projectionMatrix;
    visualization_msgs::msg::Marker model;
};

template <class T>
cv::Mat DepthModel<T>::create()
{
    this->setProjectionMatrix();
    cv::Mat dst(width, height, CV_8UC1);
    bool flag_miss(false);
    int i, j;
#pragma omp parallel for private(j)
    for (i = 0; i < width; i++)
    {
        for (j = 0; j < height; j++)
        {
            // 画素と焦点を通る直線のベクトル方程式を求める
            cv::Point3_<T> P_l, v_l;
            this->calcPixelVector(i, j, P_l, v_l);

            // 直線とモデルの交点を求める
            cv::Point3_<T> point;
            int status = this->calcIntersection(P_l, v_l, point);

            // デプス画像に登録
            if (status != 1)
                flag_miss = true;
            if (point.z > max_distance)
                dst.at<uchar>(i, j) = (uchar)(255);
            else
                dst.at<uchar>(i, j) = (uchar)(point.z / max_distance * 255);
        }
    }
    if (flag_miss)
    {
        cv::Mat nullptr_mat;
        return nullptr_mat;
    }
    return dst;
}

template <class T>
void DepthModel<T>::calcPixelVector(const int &im_x, const int &im_y,
                                    cv::Point3_<T> &P, cv::Point3_<T> &v)
{
    T P11 = this->projectionMatrix.at<T>(0, 0);
    T P12 = this->projectionMatrix.at<T>(0, 1);
    T P13 = this->projectionMatrix.at<T>(0, 2);
    T P14 = this->projectionMatrix.at<T>(0, 3);
    T P21 = this->projectionMatrix.at<T>(1, 0);
    T P22 = this->projectionMatrix.at<T>(1, 1);
    T P23 = this->projectionMatrix.at<T>(1, 2);
    T P24 = this->projectionMatrix.at<T>(1, 3);
    T P31 = this->projectionMatrix.at<T>(2, 0);
    T P32 = this->projectionMatrix.at<T>(2, 1);
    T P33 = this->projectionMatrix.at<T>(2, 2);
    T P34 = this->projectionMatrix.at<T>(2, 3);

    P.x = -1 *
          ((P14 * P33 - P13 * P34) * (P22 * P33 - P23 * P32) - (P24 * P33 - P23 * P34) * (P12 * P33 - P13 * P32)) /
          ((P11 * P33 - P13 * P31) * (P22 * P33 - P23 * P32) - (P21 * P33 - P23 * P31) * (P12 * P33 - P13 * P32));

    v.x = ((P33 * (T)im_x - P13) * (P22 * P33 - P23 * P32) - (P33 * (T)im_y - P23) * (P12 * P33 - P13 * P32)) /
          ((P11 * P33 - P13 * P31) * (P22 * P33 - P23 * P32) - (P21 * P33 - P23 * P31) * (P12 * P33 - P13 * P32));

    P.y = -1 *
          ((P14 * P33 - P13 * P34) * (P21 * P33 - P23 * P31) - (P24 * P33 - P23 * P34) * (P11 * P33 - P13 * P31)) /
          ((P12 * P33 - P13 * P32) * (P21 * P33 - P23 * P31) - (P22 * P33 - P23 * P32) * (P11 * P33 - P13 * P31));

    v.y = ((P33 * (T)im_x - P13) * (P21 * P33 - P23 * P31) - (P33 * (T)im_y - P23) * (P11 * P33 - P13 * P31)) /
          ((P12 * P33 - P13 * P32) * (P21 * P33 - P23 * P31) - (P22 * P33 - P23 * P32) * (P11 * P33 - P13 * P31));

    P.z = -(P.x * P31 / P33 + P.y * P32 / P33 + P34 / P33);

    v.z = 1. / P33 - (v.x * P31 / P33 + v.y * P32 / P33);
}

template <class T>
int DepthModel<T>::calcIntersection(const cv::Point3_<T> &P_line, const cv::Point3_<T> &v, cv::Point3_<T> &point)
{
    cv::Point3_<T> P_ellipse((T)model.pose.position.x, (T)model.pose.position.y, (T)model.pose.position.z);
    cv::Point3_<T> P = P_line - P_ellipse;
    T A = (v.x / (T)model.scale.x) * (v.x / (T)model.scale.x) +
          (v.y / (T)model.scale.y) * (v.y / (T)model.scale.y) +
          (v.z / (T)model.scale.z) * (v.z / (T)model.scale.z);
    T B = 2 * P.x * v.x / ((T)model.scale.x * (T)model.scale.x) +
          2 * P.y * v.y / ((T)model.scale.y * (T)model.scale.y) +
          2 * P.z * v.z / ((T)model.scale.z * (T)model.scale.z);
    T C = (P.x / (T)model.scale.x) * (P.x / (T)model.scale.x) +
          (P.y / (T)model.scale.y) * (P.y / (T)model.scale.y) +
          (P.z / (T)model.scale.z) * (P.z / (T)model.scale.z) - 1;

    T D = B * B - 4 * A * C;
    if (D > 0)
    {
        T t1 = (-B + std::sqrt(D)) / (2 * A);
        T t2 = (-B - std::sqrt(D)) / (2 * A);
        cv::Point3_<T> Q1 = P_line + t1 * v;
        cv::Point3_<T> Q2 = P_line + t2 * v;
        if (Q1.z > 0 && Q2.z > 0)
        {
            if (Q1.z < Q2.z)
            {
                point = Q1;
                return 1;
            }
            else
            {
                point = Q2;
                return 1;
            }
        }
        else if (Q1.z > 0 && Q2.z < 0)
        {
            point = Q1;
            return 1;
        }
        else if (Q1.z < 0 && Q2.z > 0)
        {
            point = Q2;
            return 1;
        }
        else
        {
            std::cout << " Intersection point.z < 0 " << std::endl;
            return -1;
        }
    }
    else
    {
        std::cout << "don't have Intersection" << std::endl;
        return -1;
    }
    return 0;
}

#endif