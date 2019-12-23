#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ktl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "../include/options_reconstruction.hpp"

int encoding2mat_type(const std::string &encoding)
{
    if (encoding == "mono8")
    {
        return CV_8UC1;
    }
    else if (encoding == "bgr8")
    {
        return CV_8UC3;
    }
    else if (encoding == "mono16")
    {
        return CV_16SC1;
    }
    else if (encoding == "rgba8")
    {
        return CV_8UC4;
    }
    else if (encoding == "bgra8")
    {
        return CV_8UC4;
    }
    else if (encoding == "32FC1")
    {
        return CV_32FC1;
    }
    else if (encoding == "rgb8")
    {
        return CV_8UC3;
    }
    else
    {
        throw std::runtime_error("Unsupported encoding type");
    }
}

void transformQuaternionToRotMat(
    float &m11, float &m12, float &m13,
    float &m21, float &m22, float &m23,
    float &m31, float &m32, float &m33,
    float qx, float qy, float qz, float qw)
{
    m11 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    m12 = 2.0f * qx * qy + 2.0f * qw * qz;
    m13 = 2.0f * qx * qz - 2.0f * qw * qy;

    m21 = 2.0f * qx * qy - 2.0f * qw * qz;
    m22 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    m23 = 2.0f * qy * qz + 2.0f * qw * qx;

    m31 = 2.0f * qx * qz + 2.0f * qw * qy;
    m32 = 2.0f * qy * qz - 2.0f * qw * qx;
    m33 = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image, const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm, bool show_camera, rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
    //ループカウンタ
    static int loop_count = 0;
    loop_count++;

    //Subscribe Image
    cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    if (msg_image->encoding == "rgb8")
    {
        cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
    }

    //グローバル座標とカメラ座標間の座標変換行列
    cv::Mat arm_trans(3, 1, CV_32FC1);
    cv::Mat arm_rot(3, 3, CV_32FC1);
    //並進成分
    arm_trans.at<float>(0) = msg_arm->translation.x;
    arm_trans.at<float>(1) = msg_arm->translation.y;
    arm_trans.at<float>(2) = msg_arm->translation.z;
    //回転成分
    transformQuaternionToRotMat(arm_rot.at<float>(0, 0), arm_rot.at<float>(0, 1), arm_rot.at<float>(0, 2),
                                arm_rot.at<float>(1, 0), arm_rot.at<float>(1, 1), arm_rot.at<float>(1, 2),
                                arm_rot.at<float>(2, 0), arm_rot.at<float>(2, 1), arm_rot.at<float>(2, 2),
                                msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);

    //// Key Frame 挿入 START ////
    //アーム姿勢をcv::Mat形式に変換
    static cv::Mat x1(3, 1, CV_32FC1), x2(3, 1, CV_32FC1);
    static cv::Mat r1(3, 3, CV_32FC1), r2(3, 3, CV_32FC1);
    x2 = arm_trans.clone();
    r2 = arm_rot.clone();
    if (loop_count == 1)
    {
        x1 = x2.clone();
        r1 = r2.clone();
        return;
    }

    //画像間でのカメラの移動量（運動学より）
    cv::Mat R_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
    R_arm = r2 * r1.t();
    t_arm = x2 - x1;

    //cv::imshow
    if (show_camera && loop_count % 2 == 0)
    {
        cv::Point2f center_t_arm;
        cv::Point2f p1 = cv::Point2f(msg_image->height / 2., msg_image->width / 2.);
        cv::Scalar color = cv::Scalar(0, 255, 0);
        float rotation_x = 0.0, rotation_y = PI, rotation_z = -PI / 2.;
        cv::Mat rot_x = (cv::Mat_<float>(3, 3) << 1., 0., 0., 0., std::cos(rotation_x), -std::sin(rotation_x), 0.0, std::sin(rotation_x), std::cos(rotation_x));
        cv::Mat rot_y = (cv::Mat_<float>(3, 3) << std::cos(rotation_y), 0.0, std::sin(rotation_y), 0.0, 1., 0., -std::sin(rotation_y), 0.0, std::cos(rotation_y));
        cv::Mat rot_z = (cv::Mat_<float>(3, 3) << std::cos(rotation_z), -std::sin(rotation_z), 0., std::sin(rotation_z), std::cos(rotation_z), 0., 0., 0., 1.);
        cv::Mat endo_t_arm = rot_x * rot_y * rot_z * R_arm * t_arm;
        center_t_arm = cv::Point2f(endo_t_arm.at<float>(0) * 10 + p1.x, endo_t_arm.at<float>(1) * 10 + p1.y);
        cv::arrowedLine(frame_image, p1, center_t_arm, color, 2, 8, 0, 0.5);
        cv::imshow("frame_image", frame_image);
        cv::waitKey(1);
        printf("loopcount = %d, endo_t = [%0.3f %0.3f %0.3f]\n", loop_count, endo_t_arm.at<float>(0), endo_t_arm.at<float>(1), endo_t_arm.at<float>(2));
    }
    x1 = x2.clone();
    r1 = r2.clone();
}

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    bool show_camera = false;
    size_t feature = 0;
    size_t match = 0;
    size_t prjMat = 1;

    std::string topic_sub("endoscope_image");
    std::string topic_sub_arm("arm_trans");
    std::string topic_pub("feature_point");
    std::string topic_pub_pointcloud("pointcloud");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &feature, &match, &prjMat))
    {
        return 0;
    }

    if (show_camera)
    {
        // Initialize an OpenCV named window called "cvframe".
        cv::namedWindow("frame_image", cv::WINDOW_AUTOSIZE);
    }
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("camera_vector");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());

    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);                             // Create the image publisher with our custom QoS profile.
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
    message_filters::Subscriber<geometry_msgs::msg::Transform> arm_sub(node.get(), topic_sub_arm);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync(image_sub, arm_sub, 100);
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, node_logger));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}