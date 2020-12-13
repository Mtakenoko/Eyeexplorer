#ifndef CALC_RMSE_HPP__
#define CALC_RMSE_HPP__

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

class Eye_Shape
{
public:
    Eye_Shape()
    {
        Orientation = (cv::Mat_<float>(4, 1) << 0., 0., 0., 1.);
        Position = cv::Mat::zeros(3, 1, CV_32FC1);
        Scale = (cv::Mat_<float>(3, 1) << 0.024, 0.024, 0.024);
    };
    cv::Mat Orientation;
    cv::Mat Position;
    cv::Mat Scale;
};

class Estimation_EyeBall : public rclcpp::Node
{
public:
    Estimation_EyeBall();
    cv::Mat pointcloud;
    Eye_Shape eye_model_shape;
    Eye_Shape eye_est_shape;

private:
    void initialize();
    void input_pointcloud_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);
    void input_eyeest_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeest);
    void input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball);
    void calcRMSE();
    void calcDistance();
    void topic_callback_pointcloud_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud);
    void topic_callback_eyeest_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeest);
    void topic_callback_eyeball_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball);
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_eyeest_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_eyeball_;

    bool flag_setEyeEst;
    bool flag_setPointCloud;
    bool flag_setEyeball;
};

#endif
