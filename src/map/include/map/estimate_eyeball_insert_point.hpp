#ifndef ESTIMATION_EYEBALL_HPP__
#define ESTIMATION_EYEBALL_HPP__

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include "Ceres_eyeball.hpp"

class Eye_Shape
{
public:
    Eye_Shape()
    {
        Orientation = (cv::Mat_<float>(4, 1) << 0., 0., 0., 1.);
        Position = cv::Mat::zeros(3, 1, CV_32FC1);
        Scale = cv::Mat::zeros(3, 1, CV_32FC1);
    };
    cv::Mat Orientation;
    cv::Mat Position;
    cv::Mat Scale;
};

class Estimation_EyeBall : public rclcpp::Node
{
public:
    Estimation_EyeBall();
    Eye_Shape eye_shape;
    cv::Mat pointcloud;
    cv::Point3f insert_point;

private:
    void initialize();
    void process();
    void input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    void input_marker_data(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud);
    void estimate();
    void publish();
    void topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
    void topic_callback2_(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_insertpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

Estimation_EyeBall::Estimation_EyeBall()
    : Node("eyeball_estimator")
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud", qos,
        std::bind(&Estimation_EyeBall::topic_callback_, this, std::placeholders::_1));
    subscription_insertpoint_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "pointcloud", qos,
        std::bind(&Estimation_EyeBall::topic_callback2_, this, std::placeholders::_1));
}

void Estimation_EyeBall::topic_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    Estimation_EyeBall::input_data(msg_pointcloud);
    Estimation_EyeBall::estimate();
    Estimation_EyeBall::publish();
}

void Estimation_EyeBall::topic_callback2_(const visualization_msgs::msg::Marker::SharedPtr msg_marker)
{
    Estimation_EyeBall::input_marker_data(msg_marker);
}

void Estimation_EyeBall::input_data(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud)
{
    // メンバ変数の初期化
    cv::Mat cloud(msg_pointcloud->width, 1, CV_32FC3);
    auto floatData = reinterpret_cast<float *>(msg_pointcloud->data.data());
    for (uint32_t i = 0; i < msg_pointcloud->width; i++)
    {
        for (uint32_t j = 0; j < 3; j++)
        {
            cloud.at<cv::Vec3f>(i)[j] = floatData[i * (msg_pointcloud->point_step / sizeof(float)) + j];
        }
    }
    pointcloud = cloud.clone();
}

void Estimation_EyeBall::input_marker_data(const visualization_msgs::msg::Marker::SharedPtr msg_pointcloud)
{
    insert_point.x = (float)msg_pointcloud->pose.position.x;
    insert_point.y = (float)msg_pointcloud->pose.position.y;
    insert_point.z = (float)msg_pointcloud->pose.position.z;
}

void Estimation_EyeBall::estimate()
{
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;
    //バンドル調整用パラメータ
    double ellipse_param[3];
    double ellipse_center[3];
    // 初期値
    ellipse_param[0] = 0.012;
    ellipse_param[1] = 0.012;
    ellipse_param[2] = 0.012;
    ellipse_center[0] = pointcloud.at<cv::Vec3f>(0)[0];
    ellipse_center[1] = pointcloud.at<cv::Vec3f>(0)[1];
    ellipse_center[2] = pointcloud.at<cv::Vec3f>(0)[2];

    // コスト関数
    for (int i = 0; i < pointcloud.rows; i++)
    {
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<EllipseResiduals, 1, 3, 3>(new EllipseResiduals((double)pointcloud.at<cv::Vec3f>(i)[0], (double)pointcloud.at<cv::Vec3f>(i)[1], (double)pointcloud.at<cv::Vec3f>(i)[2]));
        problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), ellipse_param, ellipse_center);
    }

    // パラメータの最適化における上限下限設定
    // 下限
    problem.SetParameterLowerBound(&ellipse_param[0], 0, 0.010);
    problem.SetParameterLowerBound(&ellipse_param[1], 0, 0.010);
    problem.SetParameterLowerBound(&ellipse_param[2], 0, 0.010);
    // 上限
    problem.SetParameterUpperBound(&ellipse_param[0], 0, 0.014);
    problem.SetParameterUpperBound(&ellipse_param[1], 0, 0.014);
    problem.SetParameterUpperBound(&ellipse_param[2], 0, 0.014);

    // problem.SetParameterBlockConstant()

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // eye_shapeに代入
    eye_shape.Position.at<float>(0) = ellipse_center[0];
    eye_shape.Position.at<float>(1) = ellipse_center[1];
    eye_shape.Position.at<float>(2) = ellipse_center[2];
    eye_shape.Scale.at<float>(0) = ellipse_param[0];
    eye_shape.Scale.at<float>(1) = ellipse_param[1];
    eye_shape.Scale.at<float>(2) = ellipse_param[2];

    // printf
    std::cout << "Position = " << eye_shape.Position << std::endl;
    std::cout << "Scale = " << eye_shape.Scale << std::endl;
}

void Estimation_EyeBall::publish()
{
    auto marker_msg = std::make_unique<visualization_msgs::msg::Marker>(); //set marker
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    int id = 0;
    marker_msg->header.frame_id = "world";
    marker_msg->header.stamp = clock->now();
    marker_msg->ns = "eye_ball";
    marker_msg->id = ++id;

    // 形状
    marker_msg->type = visualization_msgs::msg::Marker::SPHERE;
    marker_msg->action = visualization_msgs::msg::Marker::ADD;

    // 大きさ
    marker_msg->scale.x = (double)eye_shape.Scale.at<float>(0);
    marker_msg->scale.y = (double)eye_shape.Scale.at<float>(1);
    marker_msg->scale.z = (double)eye_shape.Scale.at<float>(2);

    // 色
    marker_msg->color.a = 1.0;
    marker_msg->color.r = 1.0;
    marker_msg->color.g = 0.0;
    marker_msg->color.b = 0.0;

    // 位置・姿勢
    marker_msg->pose.position.x = (double)eye_shape.Position.at<float>(0);
    marker_msg->pose.position.y = (double)eye_shape.Position.at<float>(1);
    marker_msg->pose.position.z = (double)eye_shape.Position.at<float>(2);
    marker_msg->pose.orientation.x = (double)eye_shape.Orientation.at<float>(0);
    marker_msg->pose.orientation.y = (double)eye_shape.Orientation.at<float>(1);
    marker_msg->pose.orientation.z = (double)eye_shape.Orientation.at<float>(2);
    marker_msg->pose.orientation.w = (double)eye_shape.Orientation.at<float>(3);

    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("EyeBall", 10);
    publisher_->publish(std::move(marker_msg));
    RCLCPP_INFO(this->get_logger(), "Published pointCloud!");
}
#endif
