#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stereo.hpp>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include "../include/options_stereo_vision.hpp"

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
std::string mat_type2encoding(int mat_type)
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

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image_L, const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image_R, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received LEFT image #%s", msg_image_L->header.frame_id.c_str());
  RCLCPP_INFO(logger, "Received RIGHT image #%s", msg_image_R->header.frame_id.c_str());

  //Subscribe
  cv::Mat frame_image_L(msg_image_L->height, msg_image_L->width, encoding2mat_type(msg_image_L->encoding), const_cast<unsigned char *>(msg_image_L->data.data()), msg_image_L->step);
  cv::Mat frame_image_R(msg_image_R->height, msg_image_R->width, encoding2mat_type(msg_image_R->encoding), const_cast<unsigned char *>(msg_image_R->data.data()), msg_image_R->step);

  if (msg_image_L->encoding == "rgb8")
  {
    cv::cvtColor(frame_image_L, frame_image_L, cv::COLOR_RGB2BGR);
  }
  if (msg_image_R->encoding == "rgb8")
  {
    cv::cvtColor(frame_image_R, frame_image_R, cv::COLOR_RGB2BGR);
  }

  //disparity_mapを作成する
  cv::Mat disparity_data, disparity_map, dis_data, dis_map;
  const double min = -16;
  const double max = 2544;

  //グレイスケールへ変換
  cv::Mat gr_image_L, gr_image_R;
  cv::cvtColor(frame_image_L, gr_image_L, cv::COLOR_BGR2GRAY);
  cv::cvtColor(frame_image_R, gr_image_R, cv::COLOR_BGR2GRAY);

  //StereoBMデフォルト設定
  cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();
  sbm->setNumDisparities(160);
  sbm->setBlockSize(11);
  sbm->compute(gr_image_L, gr_image_R, disparity_data);
  //cv::minMaxLoc(disparity_data, &min, &max);
  disparity_data.convertTo(disparity_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));

  //SGBM
  cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
      0,      //int minDisparity
      160,    //int numDisparities
      11,     //int SADWindowSize
      0,      //int P1 = 0
      0,      //int P2 = 0
      0,      //int disp12MaxDiff = 0
      0,      //int preFilterCap = 0
      0,      //int uniquenessRatio = 0
      100,    //int speckleWindowSize = 0
      1,      //int speckleRange = 0
      false); //bool fullDP = false

  sgbm->compute(gr_image_L, gr_image_R, dis_data);
  //cv::minMaxLoc(dis_data, &min, &max);
  dis_data.convertTo(dis_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
  cv::Mat show_map;
  cv::hconcat(disparity_map, dis_map, show_map);
  if (show_camera)
  {
    cv::imshow("stereo_vision_map", show_map);
    cv::waitKey(1);
  }
}

int main(int argc, char *argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

  //default setup
  bool show_camera = true;

  std::string topic_sub_image_L("stereo_image_L");
  std::string topic_sub_image_R("stereo_image_R");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Configure demo parameters with command line options.
  if (!parse_command_options(
          argc, argv, &depth, &reliability_policy, &history_policy, &show_camera))
  {
    return 0;
  }

  if (show_camera)
  {
    // Initialize an OpenCV named window called "cvframe".
    cv::namedWindow("stereo_vision_map", cv::WINDOW_AUTOSIZE);
  }
  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("stereo_vision");
  rclcpp::Logger node_logger = node->get_logger();

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_L(node.get(), topic_sub_image_L);
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_R(node.get(), topic_sub_image_R);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(image_sub_L, image_sub_R, 10);
  sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, node_logger));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}