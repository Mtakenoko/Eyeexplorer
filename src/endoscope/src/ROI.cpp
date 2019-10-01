#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

int
encoding2mat_type(const std::string & encoding)
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

bool g_IsFirst = true;
cv::Rect g_ROI;

/// Convert the ROS Image message to an OpenCV matrix and display it to the user.
// \param[in] msg The image message to show.
void show_image(const sensor_msgs::msg::Image::SharedPtr msg, bool show_camera, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Received image #%s", msg->header.frame_id.c_str());
  std::cerr << "Received image #" << msg->header.frame_id.c_str() << std::endl;

  if (show_camera) {
    // Convert to an OpenCV matrix by assigning the data.
    cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char *>(msg->data.data()), msg->step);

    if (msg->encoding == "rgb8") {
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    
    if(g_IsFirst){      //一回だけの処理(ROIの設定)
      g_ROI = set_ROI(frame);   //raw_imageを元にROIを決定
      g_IsFirst = false;
      printf("(ROI.x,ROI.y) = (%d,%d)\n",g_ROI.x,g_ROI.y);
      printf("(ROI.height,ROI.width) = (%d,%d)\n",g_ROI.height,g_ROI.width);
    }
    cv::Mat cvframe = frame(g_ROI); //ROIをかける

    //ガウシアンフィルタをかける
    /*cv::Mat gaussian_frame;
    cv::GaussianBlur(cvframe, cvframe, cv::Size(3,3), 10, 10);
    */
    // Show the image in a window called "showimage".
    cv::imshow("ROI", cvframe);
    // Draw the screen and wait for 1 millisecond.
    cv::waitKey(1);
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
  std::string topic("image");   

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (show_camera) {
    // Initialize an OpenCV named window called "showimage".
    cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);
    cv::waitKey(1);
  }

  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("ROI");

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
      // The history policy determines how messages are saved until taken by
      // the reader.
      // KEEP_ALL saves all messages until they are taken.
      // KEEP_LAST enforces a limit on the number of messages that are saved,
      // specified by the "depth" parameter.
      history_policy,
      // Depth represents how many messages to store in history when the
      // history policy is KEEP_LAST.
      depth
  ));

  // The reliability policy can be reliable, meaning that the underlying transport layer will try
  // ensure that every message gets received in order, or best effort, meaning that the transport
  // makes no guarantees about the order or reliability of delivery.
  qos.reliability(reliability_policy);

  auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      show_image(msg, show_camera, node->get_logger());
    };

  std::cerr << "Subscribing to topic '" << topic << "'" << std::endl;
  RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic.c_str());
  // Initialize a subscriber that will receive the ROS Image message to be displayed.
  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, qos, callback);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
