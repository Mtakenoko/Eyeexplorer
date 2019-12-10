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

#include "../include/option_cap_stereo.hpp"

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
  size_t width = 640;
  size_t height = 480;
  size_t device_L = 2;
  size_t device_R = 3;
  bool movie_mode = false;
  std::string topic_image_L("stereo_image_L");
  std::string topic_image_R("stereo_image_R");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure demo parameters with command line options.
  if (!parse_command_options(
          argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &freq, &width,
          &height, &device_L, &device_R, &movie_mode, &topic_image_L, &topic_image_R))
  {
    return 0;
  }

  // Initialize a ROS 2 node to publish images read from the OpenCV interface to the camera.
  auto node = rclcpp::Node::make_shared("cap_stereo");
  rclcpp::Logger node_logger = node->get_logger();

  // Set the parameters of the quality of service profile. Initialize as the default profile
  // and set the QoS parameters specified on the command line.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic_image_L.c_str());
  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic_image_R.c_str());

  // Create the image publisher with our custom QoS profile.
  auto pub_image_L = node->create_publisher<sensor_msgs::msg::Image>(topic_image_L, qos);
  auto pub_image_R = node->create_publisher<sensor_msgs::msg::Image>(topic_image_R, qos);

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
  auto sub = node->create_subscription<std_msgs::msg::Bool>("flip_image", rclcpp::SensorDataQoS(), callback);

  // Set a loop rate for our main event loop.
  rclcpp::WallRate loop_rate(freq);

  cv::VideoCapture cap_L, cap_R;
  if (!movie_mode)
  {
    cap_L.open(device_L);
    cap_R.open(device_R);
  }
  else
  {
    cv::String filepath_L = "/home/takeyama/workspace/ros2_ws/src/ros2/demos/image_tools/src/000001-001_Trim.mp4";
    cv::String filepath_R = "/home/takeyama/workspace/ros2_ws/src/ros2/demos/image_tools/src/000001-001_Trim.mp4";
    cap_L.open(filepath_L);
    cap_R.open(filepath_R);
  }
  // Set the width and height based on command line arguments.
  cap_L.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap_L.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
  cap_R.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap_R.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
  if (!cap_L.isOpened())
  {
    RCLCPP_ERROR(node_logger, "Could not open video stream(L cam)");
    return 1;
  }
  if (!cap_R.isOpened())
  {
    RCLCPP_ERROR(node_logger, "Could not open video stream(R cam)");
    return 1;
  }

  // Initialize OpenCV image matrices.
  cv::Mat frame_L, frame_R;
  cv::Mat flipped_frame_L, flipped_frame_R;

  RCLCPP_INFO(node_logger, "Loop Start!");
  // Our main event loop will spin until the user presses CTRL-C to exit.
  while (rclcpp::ok())
  {
    static size_t i = 1;
    // Initialize a shared pointer to an Image message.
    auto msg_L = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_R = std::make_unique<sensor_msgs::msg::Image>();
    msg_L->is_bigendian = false;
    msg_R->is_bigendian = false;
    // Get the frame from the video capture.
    cap_L >> frame_L;
    cap_R >> frame_R;
    cv::Mat pub_img_L = frame_L;
    cv::Mat pub_img_R = frame_R;

    // Check if the frame was grabbed correctly
    if (!pub_img_L.empty() && !pub_img_R.empty())
    {
      // Convert to a ROS image
      if (!is_flipped)
      {
        convert_frame_to_message(pub_img_L, i, *msg_L);
        convert_frame_to_message(pub_img_R, i, *msg_R);
      }
      else
      {
        // Flip the frame if needed
        cv::flip(pub_img_L, flipped_frame_L, 1);
        cv::flip(pub_img_R, flipped_frame_R, 1);
        convert_frame_to_message(flipped_frame_L, i, *msg_L);
        convert_frame_to_message(flipped_frame_R, i, *msg_R);
      }
      if (!show_camera == 0)
      {
        cv::Mat show_image;
        cv::hconcat(pub_img_L, pub_img_R, show_image);
        cv::imshow("cap_image_LR", show_image);
        cv::waitKey(1);
      }
      // Publish the image message and increment the frame_id.
      pub_image_L->publish(std::move(msg_L));
      pub_image_R->publish(std::move(msg_R));
      RCLCPP_INFO(node_logger, "Publishing image_L #%zd", i);
      RCLCPP_INFO(node_logger, "Publishing image_R #%zd", i);
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
