#include "../include/experiment/pullEndoscope.hpp"

#include <ctime>
#include <fstream>

PullEndoscope::PullEndoscope()
    : Node("pull_endoscope"), flag_pull(false), flag_setArm(false), flag_setEyeEst(false), flag_setPointCloud(false), flag_setEyeball(false), count(0)
{
    // QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    subscription_arm_ = this->create_subscription<geometry_msgs::msg::Transform>(
        "/endoscope_transform", qos,
        std::bind(&PullEndoscope::topic_callback_arm_, this, std::placeholders::_1));
    subscription_pointcloud_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/occupancy_grid/marker", qos,
        std::bind(&PullEndoscope::topic_callback_pointcloud_, this, std::placeholders::_1));
    subscription_eyeest_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/EyeBall", 0,
        std::bind(&PullEndoscope::topic_callback_eyeest_, this, std::placeholders::_1));
    subscription_eyeball_ = this->create_subscription<visualization_msgs::msg::Marker>(
        "/eyeball", qos,
        std::bind(&PullEndoscope::topic_callback_eyeball_, this, std::placeholders::_1));

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    std::string str(buffer);

    outputfile.open("/home/takeyama/workspace/ros2_eyeexplorer/src/experiment/output/" + str + "_distance.txt");

    outputfile << "count"
               << " "
               << "model.x"
               << " "
               << "model.y"
               << " "
               << "model.z"
               << " "
               << "est.x"
               << " "
               << "est.y"
               << " "
               << "est.z"
               << " "
               << "arm.x"
               << " "
               << "arm.y"
               << " "
               << "arm.z"
               << " "
               << "distance_model"
               << " "
               << "distance_est" << std::endl;
}

PullEndoscope::~PullEndoscope()
{
    outputfile.close();
}

void PullEndoscope::topic_callback_arm_(const geometry_msgs::msg::Transform::SharedPtr msg_arm)
{
    PullEndoscope::input_arm_data(msg_arm);
    PullEndoscope::calcDistance();
}

void PullEndoscope::topic_callback_pointcloud_(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
{
    PullEndoscope::input_pointcloud_data(msg_pointcloud);
    PullEndoscope::calcRMSE();
}

void PullEndoscope::topic_callback_eyeest_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeest)
{
    std::cout << "unko" << std::endl;
    PullEndoscope::input_eyeest_data(msg_eyeest);
    PullEndoscope::calcError();
}

void PullEndoscope::topic_callback_eyeball_(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
{
    PullEndoscope::input_eyeball_data(msg_eyeball);
}

void PullEndoscope::input_arm_data(const geometry_msgs::msg::Transform::SharedPtr msg_arm)
{
    this->arm_pose.Position.at<float>(0) = msg_arm->translation.x;
    this->arm_pose.Position.at<float>(1) = msg_arm->translation.y;
    this->arm_pose.Position.at<float>(2) = msg_arm->translation.z;
    this->arm_pose.Orientation.at<float>(0) = msg_arm->rotation.x;
    this->arm_pose.Orientation.at<float>(1) = msg_arm->rotation.y;
    this->arm_pose.Orientation.at<float>(2) = msg_arm->rotation.z;
    this->arm_pose.Orientation.at<float>(3) = msg_arm->rotation.w;
    this->flag_setArm = true;
}

void PullEndoscope::input_pointcloud_data(const visualization_msgs::msg::MarkerArray::SharedPtr msg_pointcloud)
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

void PullEndoscope::input_eyeest_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
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

void PullEndoscope::input_eyeball_data(const visualization_msgs::msg::Marker::SharedPtr msg_eyeball)
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

void PullEndoscope::calcRMSE()
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

void PullEndoscope::calcError()
{
    if (!flag_setEyeball || !flag_setEyeEst)
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

void PullEndoscope::calcDistance()
{
    if (!flag_setArm || !flag_setEyeball || !flag_setEyeEst)
        return;
    float distance_model = std::sqrt((eye_model_shape.Position.at<float>(0) - arm_pose.Position.at<float>(0)) * (eye_model_shape.Position.at<float>(0) - arm_pose.Position.at<float>(0)) +
                                     (eye_model_shape.Position.at<float>(1) - arm_pose.Position.at<float>(1)) * (eye_model_shape.Position.at<float>(1) - arm_pose.Position.at<float>(1)) +
                                     (eye_model_shape.Position.at<float>(2) - arm_pose.Position.at<float>(2)) * (eye_model_shape.Position.at<float>(2) - arm_pose.Position.at<float>(2)));
    float distance_est = std::sqrt((eye_est_shape.Position.at<float>(0) - arm_pose.Position.at<float>(0)) * (eye_est_shape.Position.at<float>(0) - arm_pose.Position.at<float>(0)) +
                                   (eye_est_shape.Position.at<float>(1) - arm_pose.Position.at<float>(1)) * (eye_est_shape.Position.at<float>(1) - arm_pose.Position.at<float>(1)) +
                                   (eye_est_shape.Position.at<float>(2) - arm_pose.Position.at<float>(2)) * (eye_est_shape.Position.at<float>(2) - arm_pose.Position.at<float>(2)));

    std::cout << std::endl;
    std::cout << "distance model : " << distance_model * 1000. << " [mm]" << std::endl;
    std::cout << "diameter est   : " << distance_est * 1000. << " [mm]" << std::endl;

    outputfile << count++ << " " << eye_model_shape.Position.at<float>(0) << " " << eye_model_shape.Position.at<float>(1) << " " << eye_model_shape.Position.at<float>(2) << " " << eye_est_shape.Position.at<float>(0) << " " << eye_est_shape.Position.at<float>(1) << " " << eye_est_shape.Position.at<float>(2) << " " << arm_pose.Position.at<float>(0) << " " << arm_pose.Position.at<float>(1) << " " << arm_pose.Position.at<float>(2) << " " << distance_model << " " << distance_est << std::endl;

    if (distance_est < THRESHOLD_PULL_DISTANCE)
        flag_pull = true;
}