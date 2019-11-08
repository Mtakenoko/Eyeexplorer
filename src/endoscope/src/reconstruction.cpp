#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include <ktl.h>

static const std::string OPENCV_WINDOW = "Image window";

cv::Mat reconstruct(std::vector<cv::DMatch> dmatch, std::vector<cv::KeyPoint> g_keypoints1, std::vector<cv::KeyPoint> g_keypoints2){
    if(dmatch.size() < 5){
        return;
    }
    std::vector<cv::Point2d> p1;
	std::vector<cv::Point2d> p2;

	//対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
	for (size_t i = 0; i < dmatch.size(); i++) {	//特徴点の数だけ処理
		cv::Mat ip(3, 1, CV_64FC1);	//3×1のベクトル
		cv::Point2d p;

		ip.at<double>(0) = g_keypoints1[dmatch[i].queryIdx].pt.x;	//1枚目の画像の特徴点のx座標を取得
		ip.at<double>(1) = g_keypoints1[dmatch[i].queryIdx].pt.y;	//1枚目の画像の特徴点のy座標を取得
		ip.at<double>(2) = 1.0;	//z座標については焦点距離1.0だとする

		cv::Mat cameraMatrix;	
		ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からグローバル平面へ写像
		p.x = ip.at<double>(0);
		p.y = ip.at<double>(1);
		p1.push_back(p);

		ip.at<double>(0) = g_keypoints2[dmatch[i].trainIdx].pt.x;	//2枚目の画像の特徴点のx座標を取得
		ip.at<double>(1) = g_keypoints2[dmatch[i].trainIdx].pt.y;	//2枚目の画像の特徴点のy座標を取得
		ip.at<double>(2) = 1.0;

		ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からグローバル平面へ写像
		p.x = ip.at<double>(0);
		p.y = ip.at<double>(1);
		p2.push_back(p);
	}
	cv::Mat mask; //RANSACの結果を保持するためのマスク
	cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003, mask);	//RANSACによって

	cv::Mat r, t;
	cv::recoverPose(essentialMat, p1, p2, r, t);

	//正規化座標系で計算しているのでProjection matrix = 外部カメラパラメータ行列
	cv::Mat prjMat1, prjMat2;
	prjMat1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
	prjMat2 = cv::Mat(3, 4, CV_64FC1);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			prjMat2.at<double>(i, j) = r.at<double>(i, j);
		}
	}
	prjMat2.at<double>(0, 3) = t.at<double>(0);
	prjMat2.at<double>(1, 3) = t.at<double>(1);
	prjMat2.at<double>(2, 3) = t.at<double>(2);
	
	cv::Mat point3D;
	cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point3D);	//三角測量
    return point3D;
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
	std::string topic_sub("ROI_image");   
	std::string topic_pub("Position");

	// Force flush of the stdout buffer.
	// This ensures a correct sync of all prints
	// even when executed simultaneously within a launch file.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	if (show_camera) {
		cv::namedWindow("reconstruction", cv::WINDOW_AUTOSIZE);  // Initialize an OpenCV named window called "showimage".
		cv::waitKey(1);
	}

	// Initialize a ROS node.
	auto node = rclcpp::Node::make_shared("ROI");
	rclcpp::Logger node_logger = node->get_logger();

	// Set quality of service profile based on command line options.
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
	qos.reliability(reliability_policy);

	auto callback = [show_camera, &node](const sensor_msgs::msg::Image::SharedPtr msg_sub){
		reconstruct(msg_sub, show_camera, node->get_logger());
	};    

	//Set QoS to Publish
	RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
	auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos); // Create the image publisher with our custom QoS profile.

	//Set QoS to Subscribe
	RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
	auto sub = node->create_subscription<sensor_msgs::msg::Image>(topic_sub, qos, callback);  // Initialize a subscriber that will receive the ROS Image message to be displayed.

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
