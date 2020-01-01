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

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/superres.hpp>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

#include "../include/option_cap_endoscope.hpp"

//画像のサイズ
#define IMAGE_HIGHT 320
#define IMAGE_WIDTH 320

//超解像初期設定(TemporalAreaRadius=4, Iterations=180)のままだと長時間を要するので質を下げる
//超解像する倍率(初期設定=4)
#define SUPER_SCALE 5
#define SUPER_TEMPORAL_AREA_RADIUS 1
#define SUPER_ITERATIONS 2

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type)
{
    switch (mat_type)
    {
    case CV_8UC1:
        return "mono8";
    case CV_8UC3:
        return "bgr8";
    case CV_16SC1:
        return "mono16";
    case CV_8UC4:
        return "rgba8";
    default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(
    const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
{
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = std::to_string(frame_id);
}

cv::Rect set_ROI(const cv::Mat src, const size_t roi_width, const size_t roi_height)
{ //ROIを設定する
    cv::Rect ROI;
    cv::Mat gray, bin;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);          //入力画像をグレースケールに変換
    cv::threshold(gray, bin, 30, 255, cv::THRESH_BINARY); //30をしきい値として二値化処理

    cv::Mat LabelImg, stats, centroids;
    int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids); //二値化処理したものにラベリング実行。戻り値はラベル数。
    if (nLab == 2)
    { //内視鏡画像と黒部分の2つだけにわけれたとき
        //int *param1 = stats.ptr<int>(1);
        ROI.height = roi_height; //param1[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT]/1.5;  //ラベリングした画像の縦の0.8倍したものをROIの縦にする
        ROI.width = roi_width;   //param1[cv::ConnectedComponentsTypes::CC_STAT_WIDTH]/1.5;    //ラベリングした画像の横の0.8倍したものをROIの横にする

        double *param2 = centroids.ptr<double>(1);
        ROI.x = static_cast<int>(param2[0]) - (int)(ROI.width / 2);  //ROIの重心位置のx
        ROI.y = static_cast<int>(param2[1]) - (int)(ROI.height / 2); //ROIの重心位置のy
    }
    else if (nLab > 2)
    { //内視鏡画像と黒部分以外にもラベルがあるとき
        int num;
        for (int i = 1; i < nLab; ++i)
        {
            int *param = stats.ptr<int>(i);
            if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 100)
            { //ラベリングした部分の大きさが100を超えたとき
                num = i;
                break;
            }
        }
        //int *param1 = stats.ptr<int>(num);
        ROI.height = roi_height; //param1[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT]/1.5;  //ラベリングした画像の縦の0.8倍したものをROIの縦にする
        ROI.width = roi_width;   //param1[cv::ConnectedComponentsTypes::CC_STAT_WIDTH]/1.5;    //ラベリングした画像の横の0.8倍したものをROIの横にする

        double *param2 = centroids.ptr<double>(num);
        ROI.x = static_cast<int>(param2[0]) - ROI.width / 2;
        ROI.y = static_cast<int>(param2[1]) - ROI.height / 2;
    }
    return ROI;
}

cv::Mat preprocess(cv::Mat src)
{
    //ヒストグラム平坦化（CLAHE）
    cv::Mat YCrCb[3], Y;
    cv::Mat clahe_img, dst_clahe;
    double clipLimit = 0.5;
    cv::Size tileGridSize = cv::Size(8, 8);

    cv::cvtColor(src, clahe_img, cv::COLOR_BGR2YCrCb); //入力画像をYCrCbに変換
    cv::split(clahe_img, YCrCb);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, tileGridSize);
    clahe->apply(YCrCb[0], YCrCb[0]);
    cv::merge(YCrCb, 3, clahe_img);
    cv::cvtColor(clahe_img, dst_clahe, cv::COLOR_YCrCb2BGR);

    //L*a*b空間でマスク処理
    cv::Mat img_Lab, Lab[3], img_mask_red, img_mask_L, img_bitwise, img_bitwise_red, img_bitwise_L, dst_mask; //L*a*b空間でマスク処理
    cv::cvtColor(dst_clahe, img_Lab, cv::COLOR_BGR2Lab);
    cv::split(img_Lab, Lab);
    cv::threshold(Lab[1], img_mask_red, 138, 255, cv::THRESH_BINARY); //下限を上げるとさらにマスクされる
    cv::bitwise_and(dst_clahe, dst_clahe, img_bitwise_red, img_mask_red);
    cv::threshold(Lab[0], img_mask_L, 180, 255, cv::THRESH_BINARY_INV); //下限を下げるとさらにマスクされる
    cv::bitwise_and(dst_clahe, dst_clahe, img_bitwise_L, img_mask_L);
    cv::bitwise_and(img_mask_red, img_mask_red, dst_mask, img_mask_L); //それぞれのマスクを足し合わせる
    cv::bitwise_and(dst_clahe, dst_clahe, img_bitwise, dst_mask);

    /*//ORB
    cv::Mat orb_img, orb_img2;
    static std::vector<cv::KeyPoint> keypoints, keypoints2;
    cv::Ptr<cv::FeatureDetector> detector, detector2;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
    detector = cv::ORB::create(100, 1.2f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
    detector2 = cv::ORB::create(100, 1.25f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
	  detector->detect(src, keypoints);
    detector2->detect(img_bitwise, keypoints2);
    cv::drawKeypoints(src, keypoints, orb_img, cv::Scalar(0,0,255));
    cv::drawKeypoints(img_bitwise, keypoints2, orb_img2, cv::Scalar(0,0,255));
    cv::imshow("ORB", orb_img);
    cv::imshow("ORB2", orb_img2);*/

    //cv::cvtColor(img_bitwise, dst_mask, cv::COLOR_Lab2BGR);
    //cv::imshow("red",img_bitwise_red);
    //cv::imshow("clahe_img", img_bitwise_L);
    //cv::imshow("mask_red",img_mask_red);
    //cv::imshow("mask_L",img_mask_L);
    //cv::imshow("total_mask", dst_mask);;
    //cv::waitKey(1);
    return dst_mask;
}

int main(int argc, char *argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t show_camera = 0;
    size_t depth = rmw_qos_profile_default.depth;
    double freq = 30.0;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    size_t width = 960;
    size_t height = 540;
    size_t roi_width = IMAGE_WIDTH;
    size_t roi_height = IMAGE_HIGHT;
    size_t device = 0;
    bool movie_mode = false;
    std::string topic("endoscope_image");
    std::string topic_mask("superres_image");

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
    auto node = rclcpp::Node::make_shared("cap_endoscope");
    rclcpp::Logger node_logger = node->get_logger();

    // Set the parameters of the quality of service profile. Initialize as the default profile
    // and set the QoS parameters specified on the command line.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
    // Create the image publisher with our custom QoS profile.
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic, qos);
    auto pub_mask = node->create_publisher<sensor_msgs::msg::Image>(topic_mask, qos);

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

    cv::VideoCapture cap;
    if (!movie_mode)
    {
        cap.open(device);
    }
    else
    {
        cv::String filepath = "/home/takeyama/workspace/ros2_ws/src/ros2/demos/image_tools/src/000001-001_Trim.mp4";
        cap.open(filepath);
    }

    // Set the width and height based on command line arguments.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node_logger, "Could not open video stream");
        return 1;
    }

    //超解像度処理クラス生成
    cv::Ptr<cv::superres::SuperResolution> superRes = cv::superres::createSuperResolution_BTVL1();

    //処理する動画を指定
    superRes->setInput(cv::superres::createFrameSource_Camera(2));
    //超解像処理の設定
    superRes->setScale(SUPER_SCALE);
    superRes->setTemporalAreaRadius(SUPER_TEMPORAL_AREA_RADIUS);
    superRes->setIterations(SUPER_ITERATIONS);

    // Initialize OpenCV image matrices.
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::Mat flipped_mask_frame;

    size_t i = 1;

    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok())
    {
        // Initialize a shared pointer to an Image message.
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        auto msg_mask = std::make_unique<sensor_msgs::msg::Image>();
        msg->is_bigendian = false;
        msg_mask->is_bigendian = false;
        // Get the frame from the video capture.
        cap >> frame;
        static cv::Rect ROI;
        static bool Is_ROI_Setted = false;
        if (!Is_ROI_Setted)
        {
            ROI = set_ROI(frame, roi_width, roi_height); //raw_imageを元にROIを決定
            Is_ROI_Setted = true;
            printf("(ROI.x,ROI.y) = (%d,%d)\n", ROI.x, ROI.y);
            printf("(ROI.height,ROI.width) = (%d,%d)\n", ROI.height, ROI.width);
        }
        cv::Mat pub_img = frame(ROI); //ROIをかける
        cv::Mat pub_mask_img = preprocess(pub_img);

        //cv::imshow("topic", pub_img);
        //cv::waitKey(1);

        // Check if the frame was grabbed correctly
        if (!pub_img.empty())
        {
            // Convert to a ROS image
            if (!is_flipped)
            {
                convert_frame_to_message(pub_img, i, *msg);
                convert_frame_to_message(pub_mask_img, i, *msg_mask);
            }
            else
            {
                // Flip the frame if needed
                cv::flip(pub_img, flipped_frame, 1);
                cv::flip(pub_mask_img, flipped_mask_frame, 1);
                convert_frame_to_message(flipped_frame, i, *msg);
                convert_frame_to_message(flipped_mask_frame, i, *msg_mask);
            }
            if (show_camera == 1)
            {
                cv::imshow("cap_endoscope", pub_img);
                cv::waitKey(1);
            }
            else if (show_camera == 2)
            {
                cv::imshow("mask", pub_mask_img);
                cv::waitKey(1);
            }
            // Publish the image message and increment the frame_id.
            pub->publish(std::move(msg));
            RCLCPP_INFO(node_logger, "Publishing image #%zd", i);
            pub_mask->publish(std::move(msg_mask));
            RCLCPP_INFO(node_logger, "Publishing mask image #%zd", i);
            ++i;
        }
        else
        {
            RCLCPP_INFO(node_logger, "pub_img is empty!");
        }
        // Do some work in rclcpp and wait for more to come in.
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
