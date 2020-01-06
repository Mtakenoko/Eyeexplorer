// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "../include/option_cap_endoscope.hpp"

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t show_camera = 0;
    size_t depth = rmw_qos_profile_default.depth;
    double freq = 1.0;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    size_t width = 960;
    size_t height = 540;
    size_t device = 0;
    bool movie_mode = false;
    std::string topic("endoscope_image");
    std::string topic_mask("mask_image");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure demo parameters with command line options.
    if (!parse_command_options(
            argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &freq, &width,
            &height, &device, &movie_mode, &topic))
    {
        return 0;
    }

    // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
    auto node = rclcpp::Node::make_shared("triangulate");
    rclcpp::Logger node_logger = node->get_logger();

    // Set the parameters of the quality of service profile. Initialize as the default profile
    // and set the QoS parameters specified on the command line.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // is_flipped will cause the incoming camera image message to flip about the y-axis.
    bool is_flipped = false;

    // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // callback.
    auto callback =
        [&is_flipped, &node_logger](const std_msgs::msg::Bool::SharedPtr msg) -> void {
        is_flipped = msg->data;
        RCLCPP_INFO(node_logger, "Set flip mode to: %s", is_flipped ? "on" : "off");
    };

    // Set the QoS profile for the subscription to the flip message.
    auto sub = node->create_subscription<std_msgs::msg::Bool>(
        "flip_image", rclcpp::SensorDataQoS(), callback);

    // Set a loop rate for our main event loop.
    rclcpp::WallRate loop_rate(freq);

    //ウィンドウの用意
    cv::namedWindow("L, R画像(基準長 = 300)");

    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok())
    {
        //ここから
        //３次元点群の座標位置を設定（８個）
        std::vector<cv::Point3f> point3D;
        point3D.push_back(cv::Point3f(-100, -100, -100));
        point3D.push_back(cv::Point3f(100, -100, -100));
        point3D.push_back(cv::Point3f(100, -100, 100));
        point3D.push_back(cv::Point3f(-100, -100, 100));
        point3D.push_back(cv::Point3f(-100, 100, -100));
        point3D.push_back(cv::Point3f(100, 100, -100));
        point3D.push_back(cv::Point3f(100, 100, 100));
        point3D.push_back(cv::Point3f(-100, 100, 100));

        //カメラの位置を設定
        float hbl = 150, hbly = 50, distance = 800;

        //カメラの内部パラメータを設定
        cv::Mat cameraMatrix = (cv::Mat_<float>(3, 3) << 320, 0, 160,
                                0, 320, 120,
                                0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);

        //回転ベクトルを設定
        cv::Mat revc = (cv::Mat_<float>(3, 1) << 0, 0, 0);
        //並進ベクトルを設定
        cv::Mat tvecL = (cv::Mat_<float>(3, 1) << hbl, hbly, distance);
        cv::Mat tvecR = (cv::Mat_<float>(3, 1) << -hbl, -hbly, distance);

        //左画像, 右画像に射影する
        std::vector<cv::Point2f> imagePointsL, imagePointsR;
        cv::projectPoints(point3D, revc, tvecL, cameraMatrix, distCoeffs, imagePointsL);
        cv::projectPoints(point3D, revc, tvecR, cameraMatrix, distCoeffs, imagePointsR);

        //printf
        std::cout << "imagePointsL" << std::endl;
        std::cout << imagePointsL << std::endl
                  << std::endl;
        std::cout << "imagePointsR" << std::endl;
        std::cout << imagePointsR << std::endl
                  << std::endl;

        //描画
        cv::Mat dst_imageL = cv::Mat(cv::Size(320, 240), CV_8UC3, cv::Scalar::all(255));
        cv::Mat dst_imageR = cv::Mat(cv::Size(320, 240), CV_8UC3, cv::Scalar::all(255));
        for (int i = 0; i < 8; i++)
        {
            cv::circle(dst_imageL, imagePointsL[i], 2, cv::Scalar::all(0), -1);
            cv::circle(dst_imageR, imagePointsR[i], 2, cv::Scalar::all(0), -1);
        }
        for (int i = 0; i < 4; i++)
        {
            cv::line(dst_imageL, imagePointsL[i], imagePointsL[(i + 1) % 4], cv::Scalar::all(0));
            cv::line(dst_imageL, imagePointsL[i + 4], imagePointsL[((i + 1) % 4) + 4], cv::Scalar::all(0));
            cv::line(dst_imageL, imagePointsL[i], imagePointsL[i + 4], cv::Scalar::all(0));
            cv::line(dst_imageR, imagePointsR[i], imagePointsR[(i + 1) % 4], cv::Scalar::all(0));
            cv::line(dst_imageR, imagePointsR[i + 4], imagePointsR[((i + 1) % 4) + 4], cv::Scalar::all(0));
            cv::line(dst_imageR, imagePointsR[i], imagePointsR[i + 4], cv::Scalar::all(0));
        }
        cv::Mat showimage;
        cv::hconcat(dst_imageL, dst_imageR, showimage);
        cv::imshow("L, R画像(基準長 = 300)", showimage);

        //左右のカメラの射影行列を設定する
        float thetaL = 0.0, thetaR = 0.0;
        cv::Mat RotL = (cv::Mat_<float>(3, 3) << std::cos(thetaL), -std::sin(thetaL), 0,
                        std::sin(thetaL), std::cos(thetaL), 0,
                        0, 0, 1);
        cv::Mat RotR = (cv::Mat_<float>(3, 3) << std::cos(thetaR), -std::sin(thetaR), 0,
                        std::sin(thetaR), std::cos(thetaR), 0,
                        0, 0, 1);
        cv::Mat tL = (cv::Mat_<float>(3, 1) << 0, 0, 0);
        cv::Mat tR = (cv::Mat_<float>(3, 1) << 2 * hbl, 2 * hbly, 0);
        cv::Mat RtL, RtR, projectionMatrixL, projectionMatrixR;
        cv::hconcat(RotL, tL, RtL);
        cv::hconcat(RotR, -tR, RtR);
        projectionMatrixL = cameraMatrix * RtL;
        projectionMatrixR = cameraMatrix * RtR;

        //三次元復元
        cv::Mat points4D;
        std::vector<cv::Point3f> point3D_result1, point3D_result2, point3D_result3;
        std::cout << "座標位置1(X, Y, Z)" << std::endl;
        for (int i = 0; i < 8; i++)
        {
            cv::triangulatePoints(projectionMatrixL, projectionMatrixR, cv::Mat(imagePointsL[i]), cv::Mat(imagePointsR[i]), points4D);
            cv::convertPointsFromHomogeneous(points4D.reshape(4, 1), point3D_result1);
            std::cout << point3D_result1 << std::endl;
        }
        std::cout << std::endl
                  << "座標位置2(X, Y, Z)" << std::endl;
        cv::triangulatePoints(projectionMatrixL, projectionMatrixR, cv::Mat(imagePointsL), cv::Mat(imagePointsR), points4D);
        cv::convertPointsFromHomogeneous(points4D.reshape(4, 1), point3D_result2);
        std::cout << point3D_result2 << std::endl
                  << std::endl;
        std::cout << "座標位置3(X, Y, Z)" << std::endl;
        cv::triangulatePoints(projectionMatrixL, projectionMatrixR, imagePointsL, imagePointsR, points4D);
        cv::convertPointsFromHomogeneous(points4D.reshape(4, 1), point3D_result3);
        std::cout << point3D_result3 << std::endl
                  << std::endl;
        cv::waitKey(3);

        //ここまで
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
