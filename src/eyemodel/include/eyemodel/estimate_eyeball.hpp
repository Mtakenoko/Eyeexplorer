#ifndef ESTIMATION_EYEBALL_HPP__
#define ESTIMATION_EYEBALL_HPP__

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "Ceres_eyeball.hpp"

#define THRESHOLD_NUM_POINTS 10

class Eye_Shape
{
public:
    Eye_Shape()
    {
        Orientation = (cv::Mat_<double>(4, 1) << 0., 0., 0., 1.);
        Position = cv::Mat::zeros(3, 1, CV_64FC1);
        Scale = (cv::Mat_<double>(3, 1) << 0.024, 0.024, 0.024);
    };
    cv::Mat Orientation;
    cv::Mat Position;
    cv::Mat Scale;
};

class Estimation_EyeBall
{
public:
    Estimation_EyeBall();
    Eye_Shape eye_shape;
    cv::Mat pointcloud;
    cv::Point3d insert_point;

public:
    void topic_callback_(const geometry_msgs::msg::Transform::SharedPtr msg);
    void setInputflag();
    void setCalcflag();
    int getPointCloudNum();
    void cancel();

private:
    void initialize();
    void process();
    void input_data(const geometry_msgs::msg::Transform::SharedPtr msg);
    void estimate_ceres();
    void publish();

    bool flag_setPoint;
    bool flag_setPointCloud;
    bool flag_setCalc;
    bool flag_publish;
};
#endif
