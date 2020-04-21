#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "../include/options_reconstruction_online_delay.hpp"
#include "../../HTL/include/transform.h"
#include "../../HTL/include/msg_converter.h"

Transform transform;
Converter converter;

cv::Mat dst_frame, pre_dst_frame;
std::vector<cv::KeyPoint> keypoints_frame;
std::vector<cv::Point2f> points_frame;

void KLT_checker(const sensor_msgs::msg::Image::SharedPtr msg_image, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud>> pub_feature, size_t feature,
                 rclcpp::Logger logger, std::shared_ptr<rclcpp::Node> node)
{
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  static long int loop_counter = 0;
  static bool Initializer = true;

  //Subscribe
  cv::Mat frame_image(msg_image->height, msg_image->width, CV_8UC3, const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
  cv::cvtColor(frame_image, dst_frame, cv::COLOR_BGR2GRAY);

  // Publish
  sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
  sensor_msgs::msg::ChannelFloat32 id_of_point;

  // CLAHEによるヒストグラム平坦化
  cv::Mat dst_frame2, show_image;
  auto clahe = cv::createCLAHE(20.0);
  clahe->apply(dst_frame, dst_frame);
  // cv::hconcat(dst_frame, dst_frame2, show_image);
  // cv::imshow("clahe", show_image);

  //Initialize
  if (Initializer || loop_counter % 150 == 0)
  {
    Initializer = false;

    ////  detection 開始  ////
    cv::Ptr<cv::FeatureDetector> detector;

    if (feature == 0)
    {
      // AKAZE特徴抽出
      detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00001f); // 検出器（自分で設定）
    }
    else if (feature == 1)
    {
      //ORB特徴量抽出
      detector = cv::ORB::create(500, 1.2f, 8, 10, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
    }
    else if (feature == 2)
    {
      //BRISK特徴量抽出
      detector = cv::BRISK::create(120, 3, 0.6f);
    }
    else
    {
      printf("Choosing Incorrect Option of Feature point detector.\n");
      return;
    }
    detector->detect(dst_frame, keypoints_frame);

    // std::vector<Point2f>へおひっこし
    std::vector<cv::KeyPoint>::iterator it = keypoints_frame.begin();
    std::vector<cv::KeyPoint>::iterator it_end = keypoints_frame.end();
    points_frame.clear();
    for (; it != it_end; it++)
    {
      cv::Point2f p;
      p.x = it->pt.x;
      p.y = it->pt.y;
      points_frame.push_back(p);
    }

    pre_dst_frame = dst_frame.clone();
    ++loop_counter;
    printf("Initialized\n");
    return;
  }

  if (points_frame.empty())
    return;

  // KLTトラッキング
  std::vector<cv::Point2f> points_frame_KLT;
  std::vector<uchar> status_KLT;
  std::vector<float> err_KLT;
  cv::calcOpticalFlowPyrLK(pre_dst_frame, dst_frame, points_frame, points_frame_KLT, status_KLT, err_KLT, cv::Size(64, 64), 3,
                           cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01), 0, 0.01);

  // 追跡できないもの、移動量が大きすぎるもの、静止しているものは特徴点をリストから削除
  // 静止しているものを除去するお陰で、変な蜂の巣状フィルタを除外することができる
  size_t i, k;
  for (i = k = 0; i < status_KLT.size(); i++)
  {
    double dist = cv::norm(points_frame[i] - points_frame_KLT[i]);
    if (status_KLT[i] == 0)
      continue;
    if (dist > 10 || dist < 0.5)
      continue;
    points_frame[k] = points_frame[i];
    points_frame_KLT[k++] = points_frame_KLT[i];

    // point
    geometry_msgs::msg::Point32 point;
    point.x = points_frame[k].x;
    point.y = points_frame[k].y;
    point.z = 1;
    feature_points->points.push_back(point);
  }
  points_frame.resize(k);
  points_frame_KLT.resize(k);

  // 追跡点が減り過ぎたら特徴点の初期化
  if (points_frame_KLT.size() < 10)
  {
    Initializer = true;
    return;
  }

  //　特徴点をcircleで描く
  cv::Mat image_KTL;
  image_KTL = frame_image.clone();
  for (int i = 0; i < points_frame_KLT.size(); i++)
  {
    cv::Scalar c(i * 255 / points_frame_KLT.size(), 255, 255 - i * 255 / points_frame_KLT.size());
    cv::circle(frame_image, points_frame_KLT[i], 3, c, -1, cv::LINE_AA);
  }
  cv::imshow("cvframe", frame_image);
  cv::waitKey(10);

  // // 実際に点の対応どんな感じよ
  // for (int i = 0; i < points_frame_KLT.size(); i++)
  // {
  //   printf("(%f, %f) , (%f, %f)\n", points_frame[i].x, points_frame[i].y, points_frame_KLT[i].x, points_frame_KLT[i].y );
  // }

  //frame→frame2に移行
  pre_dst_frame = dst_frame.clone();
  points_frame.clear();
  std::copy(points_frame_KLT.begin(), points_frame_KLT.end(), std::back_inserter(points_frame));

  // Publish
  feature_points->header = std_msgs::msg::Header();
  feature_points->header.stamp = rclcpp::Clock().now();
  feature_points->header.frame_id = msg_image->header.frame_id;
  feature_points->channels.push_back(id_of_point);

  pub_feature->publish(std::move(feature_points));

  ++loop_counter;
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
  std::string topic_pub_pointcloud("feature");

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
    cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
  }
  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("KLT_checker");
  rclcpp::Logger node_logger = node->get_logger();

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  auto pub_feature = node->create_publisher<sensor_msgs::msg::PointCloud>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

  auto callback = [pub_feature, feature, &node](const sensor_msgs::msg::Image::SharedPtr msg_sub) {
    KLT_checker(msg_sub, pub_feature, feature, node->get_logger(), node);
  };

  //Set QoS to Subscribe
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}