#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <ktl.h>

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

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
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

cv::Rect set_ROI(const cv::Mat src){    //ROIを設定する
    
  cv::Rect ROI;
  cv::Mat gray, bin;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);   //入力画像をグレースケールに変換
  cv::threshold(gray, bin, 100, 255, cv::THRESH_BINARY);   //30をしきい値として二値化処理
       
  cv::Mat LabelImg, stats, centroids;
  int nLab = cv::connectedComponentsWithStats(bin, LabelImg, stats, centroids);   //二値化処理したものにラベリング実行。戻り値はラベル数。
  if (nLab == 2 ) {    //内視鏡画像と黒部分の2つだけにわけれたとき
    int *param1 = stats.ptr<int>(1); 
    ROI.height = param1[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT]/1.5;  //ラベリングした画像の縦の0.8倍したものをROIの縦にする
    ROI.width = param1[cv::ConnectedComponentsTypes::CC_STAT_WIDTH]/1.5;    //ラベリングした画像の横の0.8倍したものをROIの横にする

  	double *param2 = centroids.ptr<double>(1);
   	ROI.x = static_cast<int>(param2[0]) - (int)(ROI.width / 2);    //ROIの重心位置のx
    ROI.y = static_cast<int>(param2[1]) - (int)(ROI.height / 2);   //ROIの重心位置のy  
  } 
  else if (nLab > 2){  //内視鏡画像と黒部分以外にもラベルがあるとき
    int num;
    for (int i = 1; i < nLab; ++i) {
	    int *param = stats.ptr<int>(i);
      if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 100) {  //ラベリングした部分の大きさが100を超えたとき
		    num = i;
		    break;
	    }
	  }
	  int *param1 = stats.ptr<int>(num);
	  ROI.height = param1[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT]/1.5;
    ROI.width = param1[cv::ConnectedComponentsTypes::CC_STAT_WIDTH]/1.5;

	  double *param2 = centroids.ptr<double>(num);
	  ROI.x = static_cast<int>(param2[0]) - ROI.width / 2;
	  ROI.y = static_cast<int>(param2[1]) - ROI.height / 2; 
  }
  return ROI;
}

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> g_pub;
void subpub_image(const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  if (show_camera) {
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);

    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    static bool IsFirst = true;
    static cv::Rect ROI;
    if(IsFirst){      //一回だけの処理(ROIの設定)
      ROI = set_ROI(frame);   //raw_imageを元にROIを決定
      IsFirst = false;
      printf("(ROI.x,ROI.y) = (%d,%d)\n",ROI.x,ROI.y);
      printf("(ROI.height,ROI.width) = (%d,%d)\n",ROI.height,ROI.width);
    }
    cv::Mat cvframe = frame(ROI); //ROIをかける

    //ガウシアンフィルタをかける
    cv::Mat gaussian_frame;
    //cv::GaussianBlur(cvframe, cvframe, cv::Size(3,3), 10, 10);
    
    // Show the image in a window called "showimage".
    cv::imshow("ROI", cvframe);
    // Draw the screen and wait for 1 millisecond.
    cv::waitKey(1);

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

    convert_frame_to_message(cvframe, atoi(msg->header.frame_id.c_str()), *msg_pub);  //cv → msg
    g_pub->publish(std::move(msg_pub)); 
  }
}

int main(int argc, char * argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  bool show_camera = true;
  std::string topic_sub("image");   
  std::string topic_pub("ROI_image");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (show_camera) {
    cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);  // Initialize an OpenCV named window called "showimage".
    cv::waitKey(1);
  }

  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("ROI");
  rclcpp::Logger node_logger = node->get_logger();

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg_sub){
    subpub_image(msg_sub, show_camera, node->get_logger());
  };    

  //Set QoS to Subscribe
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, callback);  // Initialize a subscriber that will receive the ROS Image message to be displayed.

  //Set QoS to Publish
  RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
  g_pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos); // Create the image publisher with our custom QoS profile.

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
