#include "../include/experiment/calcRMSE.hpp"

Estimation_EyeBall::Estimation_EyeBall()
    : Node("eyeball_estimator_insertion_point"), flag_setEyeEst(false), flag_setPointCloud(false), flag_setEyeball(false)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_pointcloud_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/occupancy_grid/marker", qos,
        std::bind(&Estimation_EyeBall::topic_callback_pointcloud_, this, std::placeholders::_1));
    subscription_eyeest_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/EyeBall", qos,
        std::bind(&Estimation_EyeBall::topic_callback_eyeest_, this, std::placeholders::_1));
    subscription_eyeball_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/eyeball", qos,
        std::bind(&Estimation_EyeBall::topic_callback_eyeball_, this, std::placeholders::_1));
}

void Estimation_EyeBall::topic_callback_pointcloud_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    Estimation_EyeBall::input_pointcloud_data(msg_pointcloud);
    Estimation_EyeBall::calcRMSE();
}

void Estimation_EyeBall::topic_callback_eyeest_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeest)
{
    Estimation_EyeBall::input_eyeest_data(msg_eyeest);
    Estimation_EyeBall::calcDistance();
}

void Estimation_EyeBall::topic_callback_eyeball_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    Estimation_EyeBall::input_eyeball_data(msg_eyeball);
}

void Estimation_EyeBall::input_pointcloud_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    // メンバ変数の初期化
    cv::Mat cloud(msg_pointcloud->markers.size(), 1, CV_32FC3);
    for (size_t i = 0; i < msg_pointcloud->markers.size(); i++)
    {
        cloud.at<cv::Vec3f>(i)[0] = msg_pointcloud->markers[i].pose.position.x;
        cloud.at<cv::Vec3f>(i)[1] = msg_pointcloud->markers[i].pose.position.y;
        cloud.at<cv::Vec3f>(i)[2] = msg_pointcloud->markers[i].pose.position.z;
    }
    this->pointcloud = cloud.clone();
    flag_setPointCloud = true;
}

void Estimation_EyeBall::input_eyeest_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    eye_est_shape.Position.at<float>(0) = (float)msg_eyeball->pose.position.x;
    eye_est_shape.Position.at<float>(1) = (float)msg_eyeball->pose.position.y;
    eye_est_shape.Position.at<float>(2) = (float)msg_eyeball->pose.position.z;
    eye_est_shape.Orientation.at<float>(0) = (float)msg_eyeball->pose.orientation.x;
    eye_est_shape.Orientation.at<float>(1) = (float)msg_eyeball->pose.orientation.y;
    eye_est_shape.Orientation.at<float>(2) = (float)msg_eyeball->pose.orientation.z;
    eye_est_shape.Orientation.at<float>(3) = (float)msg_eyeball->pose.orientation.w;
    eye_est_shape.Scale.at<float>(0) = (float)msg_eyeball->scale.x;
    eye_est_shape.Scale.at<float>(1) = (float)msg_eyeball->scale.y;
    eye_est_shape.Scale.at<float>(2) = (float)msg_eyeball->scale.z;
    flag_setEyeEst = true;
}

void Estimation_EyeBall::input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    eye_model_shape.Position.at<float>(0) = (float)msg_eyeball->pose.position.x;
    eye_model_shape.Position.at<float>(1) = (float)msg_eyeball->pose.position.y;
    eye_model_shape.Position.at<float>(2) = (float)msg_eyeball->pose.position.z;
    eye_model_shape.Orientation.at<float>(0) = (float)msg_eyeball->pose.orientation.x;
    eye_model_shape.Orientation.at<float>(1) = (float)msg_eyeball->pose.orientation.y;
    eye_model_shape.Orientation.at<float>(2) = (float)msg_eyeball->pose.orientation.z;
    eye_model_shape.Orientation.at<float>(3) = (float)msg_eyeball->pose.orientation.w;
    eye_model_shape.Scale.at<float>(0) = (float)msg_eyeball->scale.x;
    eye_model_shape.Scale.at<float>(1) = (float)msg_eyeball->scale.y;
    eye_model_shape.Scale.at<float>(2) = (float)msg_eyeball->scale.z;
    flag_setEyeball = true;
}

void Estimation_EyeBall::calcRMSE()
{
    if (!flag_setPointCloud || !flag_setEyeball)
        return;

    // 点群とモデル眼球との位置精度の評価としてRMSEを計算する
    float RMSE = 0.;
    for (int i = 0; i < pointcloud.rows; i++)
    {
        float distance_from_center = std::sqrt((eye_model_shape.Position.at<float>(0) - pointcloud.at<cv::Vec3f>(i)[0]) * (eye_model_shape.Position.at<float>(0) - pointcloud.at<cv::Vec3f>(i)[0]) +
                                               (eye_model_shape.Position.at<float>(1) - pointcloud.at<cv::Vec3f>(i)[1]) * (eye_model_shape.Position.at<float>(1) - pointcloud.at<cv::Vec3f>(i)[1]) +
                                               (eye_model_shape.Position.at<float>(2) - pointcloud.at<cv::Vec3f>(i)[2]) * (eye_model_shape.Position.at<float>(2) - pointcloud.at<cv::Vec3f>(i)[2]));
        float error = distance_from_center - eye_model_shape.Scale.at<float>(0) / 2.0;
        RMSE += error * error;
    }
    RMSE /= pointcloud.rows;
    RMSE = std::sqrt(RMSE);

    std::cout << std::endl;
    std::cout << "pointcloud nums : " << pointcloud.rows << std::endl;
    std::cout << "RMSE : " << RMSE * 1000 << " [mm]" << std::endl;
}

void Estimation_EyeBall::calcDistance()
{
    if (!flag_setEyeball && !flag_setEyeEst)
        return;

    float distance_center = std::sqrt((eye_model_shape.Position.at<float>(0) - eye_est_shape.Position.at<float>(0)) * (eye_model_shape.Position.at<float>(0) - eye_est_shape.Position.at<float>(0)) +
                                      (eye_model_shape.Position.at<float>(1) - eye_est_shape.Position.at<float>(1)) * (eye_model_shape.Position.at<float>(1) - eye_est_shape.Position.at<float>(1)) +
                                      (eye_model_shape.Position.at<float>(2) - eye_est_shape.Position.at<float>(2)) * (eye_model_shape.Position.at<float>(2) - eye_est_shape.Position.at<float>(2)));
    float diameter_error = std::sqrt((eye_model_shape.Scale.at<float>(0) - eye_est_shape.Scale.at<float>(0)) * (eye_model_shape.Scale.at<float>(0) - eye_est_shape.Scale.at<float>(0)) +
                                     (eye_model_shape.Scale.at<float>(1) - eye_est_shape.Scale.at<float>(1)) * (eye_model_shape.Scale.at<float>(1) - eye_est_shape.Scale.at<float>(1)) +
                                     (eye_model_shape.Scale.at<float>(2) - eye_est_shape.Scale.at<float>(2)) * (eye_model_shape.Scale.at<float>(2) - eye_est_shape.Scale.at<float>(2)));

    std::cout << std::endl;
    std::cout << "distance center : " << distance_center * 1000. << " [mm]" << std::endl;
    std::cout << "diameter error : " << diameter_error * 1000. << " [mm]" << std::endl;
}