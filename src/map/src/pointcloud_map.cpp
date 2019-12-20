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

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    bool show_camera = false;

    std::string topic_sub("pointcloud");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    if (show_camera)
    {
        // Initialize an OpenCV named window called "cvframe".
        cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
    }
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("pointcloud");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //グローバル座標とカメラ座標間の座標変換行列
    cv::Mat arm_trans(3, 1, CV_32FC1);
    cv::Mat arm_rot(3, 3, CV_32FC1);
    auto callback_arm_trans = [&arm_trans, &arm_rot, &node](const geometry_msgs::msg::Transform::SharedPtr msg_sub) {
        //並進成分
        arm_trans.at<float>(0) = msg_sub->translation.x;
        arm_trans.at<float>(1) = msg_sub->translation.y;
        arm_trans.at<float>(2) = msg_sub->translation.z;
        //回転成分
        transformQuaternionToRotMat(arm_rot.at<float>(0, 0), arm_rot.at<float>(1, 0), arm_rot.at<float>(2, 0),
                                    arm_rot.at<float>(0, 1), arm_rot.at<float>(1, 1), arm_rot.at<float>(2, 1),
                                    arm_rot.at<float>(0, 2), arm_rot.at<float>(1, 2), arm_rot.at<float>(2, 2),
                                    msg_sub->rotation.x, msg_sub->rotation.y, msg_sub->rotation.z, msg_sub->rotation.w);
        //printf("arm_trans = [%0.2f %0.2f %0.2f]\n", arm_trans.at<float>(0), arm_trans.at<float>(1), arm_trans.at<float>(2));
        //printf("arm_rot = [%0.2f %0.2f %0.2f]\n          %0.2f %0.2f %0.2f\n          %0.2f %0.2f %0.2f]\n", arm_rot.at<float>(0, 0), arm_rot.at<float>(0, 1), arm_rot.at<float>(0, 2), arm_rot.at<float>(1, 0), arm_rot.at<float>(1, 1), arm_rot.at<float>(1, 2), arm_rot.at<float>(2, 0), arm_rot.at<float>(2, 1), arm_rot.at<float>(2, 2));
    };
    auto sub_arm = node->create_subscription<geometry_msgs::msg::Transform>(topic_sub_arm, qos, callback_arm_trans);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}