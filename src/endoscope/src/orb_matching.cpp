#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"

#include "../include/options_orb_matching.hpp"

int encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}
std::string mat_type2encoding(int mat_type)
{
  switch (mat_type) {
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
void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg)
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

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> & msg_image, const std::shared_ptr<const sensor_msgs::msg::Image> & msg_mask
  , bool show_camera, size_t feature, size_t match, bool mask, rclcpp::Logger logger
  , std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub)
  {
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  // Convert to an OpenCV matrix by assigning the data.
  cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
  cv::Mat frame_mask(msg_mask->height, msg_mask->width, encoding2mat_type(msg_mask->encoding), const_cast<unsigned char *>(msg_mask->data.data()), msg_mask->step);

  if (msg_image->encoding == "rgb8") {
  cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
  }
  if (msg_mask->encoding == "rgb8") {
  cv::cvtColor(frame_mask, frame_mask, cv::COLOR_RGB2BGR);
  }

  ////  detection 開始  ////
  static cv::Mat dst1, dst2, mask1, mask2;
  static std::vector<cv::KeyPoint> keypoints1, keypoints2;
  static cv::Mat descriptor1, descriptor2;

  dst2 = frame_image.clone();
  mask2 = frame_mask.clone();
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
  if (feature == 0){
      // AKAZE特徴抽出
      detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0001f); // 検出器（自分で設定）
      descriptorExtractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0001f);
  } else if (feature == 1){
      //ORB特徴量抽出
      detector = cv::ORB::create(10000, 1.2f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
      descriptorExtractor = cv::ORB::create(10000, 1.2f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
  } else if (feature == 2){     
      //BRISK特徴量抽出
      detector = cv::BRISK::create(120, 3, 0.6f);
      descriptorExtractor = cv::BRISK::create();  
  } else{
      printf("Choosing Incorrect Option of Feature point detector.\n");
      return;
  }
  detector->detect(dst2, keypoints2);
  descriptorExtractor->compute(dst2, keypoints2, descriptor2);

  static int i = 0; i++;
  if(i < 3){
      dst1 = dst2.clone();
      keypoints1 = keypoints2;
      descriptor1 = descriptor2.clone();    
      mask1 = mask2.clone();
      return;
  }
  ////    detection 終了  ////

  ////    matching 開始   ////
  //対応点の探索
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (match == 0){
      //Brute-Force matcher
      matcher = cv::DescriptorMatcher::create("BruteForce");   
  }else if (match == 1){
      //FLANN
      matcher = cv::DescriptorMatcher::create("FlannBased");  
  }else{
      printf("Choosing Incorrect Option of Matcher\n");
      return;
  }

  std::vector<cv::DMatch> dmatch;
  std::vector<cv::DMatch> dmatch12, dmatch21;
  std::vector<cv::Point2f> match_point1,match_point2;
  if(mask){ //maskをかける場合
    //maskの領域生成
    cv::Mat mask12 = cv::Mat::zeros((keypoints1.size()), keypoints2.size(), CV_8U);
    cv::Mat mask21 = cv::Mat::zeros((keypoints2.size()), keypoints1.size(), CV_8U);
    int count = 0;
    if(!mask1.empty() && !mask2.empty()){
      cv::Point2d pt1, pt2;
      for(size_t i = 0; i < keypoints1.size(); i++){
        for(size_t j = 0; j < keypoints2.size(); j++){
          pt1 = keypoints1[i].pt;
          pt2 = keypoints2[j].pt;
          if(mask1.at<uchar>(pt1) == 255 && mask2.at<uchar>(pt2) == 255){ //キーポイント点ではどちらもマスクが白なら
            mask12.at<uchar>(i, j) = 255; //その点についてはマスクを除去してあげよう
            count++;
          }
        }
      }
    }
    printf("count num = %d\n",count);
    mask21 = mask12.t();
    if(!mask12.empty() && !mask21.empty()){
      matcher->match(descriptor1, descriptor2, dmatch12, mask12); //dst1 -> dst2
      matcher->match(descriptor2, descriptor1, dmatch21, mask21); //dst2 -> dst1
    }else{
      matcher->match(descriptor1, descriptor2, dmatch12); //dst1 -> dst2
      matcher->match(descriptor2, descriptor1, dmatch21); //dst2 -> dst1
    }
  } else {  //maskをかけない場合
    matcher->match(descriptor1, descriptor2, dmatch12); //dst1 -> dst2
    matcher->match(descriptor2, descriptor1, dmatch21); //dst2 -> dst1
  }

  //dst1 -> dst2 と dst2 -> dst1の結果が一致しているか検証
	int good_count = 0;
	for (int i = 0; i < (int)dmatch12.size(); i++) {		
		cv::DMatch m12 = dmatch12[i];
		cv::DMatch m21 = dmatch21[m12.trainIdx];

		if (m21.trainIdx == m12.queryIdx) {// 一致しているものだけを抜粋
			dmatch.push_back(m12);
			match_point1.push_back(keypoints1[dmatch12[i].queryIdx].pt);
			match_point2.push_back(keypoints2[dmatch12[i].trainIdx].pt);
			good_count++;
		}
	}
	//ホモグラフィ行列推定
	const int MIN_MATCH_COUNT = 10;
	if (good_count > MIN_MATCH_COUNT) { //十分対応点が見つかるならば
		cv::Mat masks;
		cv::Mat H = cv::findHomography(match_point1, match_point2, masks, cv::RANSAC, 0.01);

		//RANSACで使われた対応点のみ抽出
		for (int i = 0; i < masks.rows; i++) {
			uchar *inliner = masks.ptr<uchar>(i);
			if (inliner[0] == 1) {
				dmatch.push_back(dmatch[i]);
			}
		}
	}     
        
  //インライアの対応点のみ表示
  cv::Mat cvframe;
  cv::drawMatches(dst1, keypoints1, dst2, keypoints2, dmatch, cvframe);
  if (show_camera){
      cv::imshow("cvframe", cvframe);
      cv::waitKey(1);
  }

  //
  dst1 = dst2.clone();
  keypoints1 = keypoints2;
  descriptor1 = descriptor2.clone(); 
  mask1 = mask2.clone();  

  ////    matching 終了   ////
  //Publish Image
  RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
  auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();

  convert_frame_to_message(cvframe, atoi(msg_image->header.frame_id.c_str()), *msg_pub);  //cv → msg
  pub->publish(std::move(msg_pub)); 

}

int main(int argc, char * argv[])
{
    // Pass command line arguments to rclcpp.
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    //std::string topic_sub("preprocessed_image");   
    std::string topic_sub("endoscope_image");   
    std::string topic_sub_mask("mask_image");   
    std::string topic_pub("feature_point");
    bool show_camera = false;
    size_t feature = 0;
    size_t match = 0;
    bool mask = false;

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure demo parameters with command line options.
    if (!parse_command_options(
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &feature, &match, &mask))
    {
        return 0;
    }

    if (show_camera) {
    // Initialize an OpenCV named window called "cvframe".
    cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
    cv::waitKey(1);
    }

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("orb_matching");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos); // Create the image publisher with our custom QoS profile.

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
    message_filters::Subscriber<sensor_msgs::msg::Image> mask_image_sub(node.get(), topic_sub_mask);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(image_sub, mask_image_sub, 10);
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, mask, node_logger, pub));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}