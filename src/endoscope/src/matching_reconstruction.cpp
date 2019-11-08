#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "vector"

#include <ktl.h>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"

#include "rclcpp/rclcpp.hpp"


#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

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

void convert_point3D_to_PCL(const cv::Mat & point4D, size_t frame_id, sensor_msgs::msg::PointCloud2 & msg){
  msg.height = point4D.rows;
  msg.width = point4D.cols;
  memccpy(&msg.data[0], point4D.data, point4D.rows, point4D.cols);
  msg.header.frame_id = std::to_string(frame_id);
}

void transformQuaternionToRotMat(
    float &m11, float &m12, float &m13,
    float &m21, float &m22, float &m23,
    float &m31, float &m32, float &m33,
    float qx, float qy, float qz, float qw
) {
    m11 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
    m12 = 2.0f * qx * qy + 2.0f * qw * qz;
    m13 = 2.0f * qx * qz - 2.0f * qw * qy;

    m21 = 2.0f * qx * qy - 2.0f * qw * qz;
    m22 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
    m23 = 2.0f * qy * qz + 2.0f * qw * qx;

    m31 = 2.0f * qx * qz + 2.0f * qw * qy;
    m32 = 2.0f * qy * qz - 2.0f * qw * qx;
    m33 = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
}
bool transformRotMatToQuaternion(
    float &qx, float &qy, float &qz, float &qw,
    float m11, float m12, float m13,
    float m21, float m22, float m23,
    float m31, float m32, float m33
) {
    // 最大成分を検索
    float elem[ 4 ]; // 0:x, 1:y, 2:z, 3:w
    elem[ 0 ] = m11 - m22 - m33 + 1.0f;
    elem[ 1 ] = -m11 + m22 - m33 + 1.0f;
    elem[ 2 ] = -m11 - m22 + m33 + 1.0f;
    elem[ 3 ] = m11 + m22 + m33 + 1.0f;

    unsigned biggestIndex = 0;
    for ( int i = 1; i < 4; i++ ) {
        if ( elem[i] > elem[biggestIndex] )
            biggestIndex = i;
    }

    if ( elem[biggestIndex] < 0.0f )
        return false; // 引数の行列に間違いあり！

    // 最大要素の値を算出
    float *q[4] = {&qx, &qy, &qz, &qw};
    float v = sqrtf( elem[biggestIndex] ) * 0.5f;
    *q[biggestIndex] = v;
    float mult = 0.25f / v;

    switch ( biggestIndex ) {
    case 0: // x
        *q[1] = (m12 + m21) * mult;
        *q[2] = (m31 + m13) * mult;
        *q[3] = (m23 - m32) * mult;
        break;
    case 1: // y
        *q[0] = (m12 + m21) * mult;
        *q[2] = (m23 + m32) * mult;
        *q[3] = (m31 - m13) * mult;
        break;
    case 2: // z
        *q[0] = (m31 + m13) * mult;
        *q[1] = (m23 + m32) * mult;
        *q[3] = (m12 - m21) * mult;
    break;
    case 3: // w
        *q[0] = (m23 - m32) * mult;
        *q[1] = (m31 - m13) * mult;
        *q[2] = (m12 - m21) * mult;
        break;
    }
    return true;
}
void convertCVMattoKtlMatrix(cv::Mat r, Ktl::Matrix<3, 3> &R){
  for (int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      R[i][j] = r.at<float>(i, j); 
    }
  }
}
void convertCVMattoKtlMatrix2(
    float &R11, float &R12, float &R13,
    float &R21, float &R22, float &R23,
    float &R31, float &R32, float &R33,
    float r11,  float r12,  float r13,
    float r21,  float r22,  float r23,
    float r31,  float r32,  float r33){
    R11 = r11; R12 = r12; R13 = r13;
    R21 = r21; R22 = r22; R23 = r23;
    R31 = r31; R32 = r32; R33 = r33;
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> & msg_image, const std::shared_ptr<const sensor_msgs::msg::Image> & msg_mask
  , bool show_camera, size_t feature, size_t match, bool mask, cv::viz::Viz3d visualizeWindow, cv::Mat arm_trans, cv::Mat arm_rot, rclcpp::Logger logger 
  , std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud, std::shared_ptr<rclcpp::Node> node)
  {
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  
  //Subscribe
  cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
  cv::Mat frame_mask(msg_mask->height, msg_mask->width, encoding2mat_type(msg_mask->encoding), const_cast<unsigned char *>(msg_mask->data.data()), msg_mask->step);

  if (msg_image->encoding == "rgb8") {
  cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
  }
  if (msg_mask->encoding == "rgb8") {
  cv::cvtColor(frame_mask, frame_mask, cv::COLOR_RGB2BGR);
  }

  //アーム姿勢をcv::Mat形式に変換
  static cv::Mat x1(3, 1, CV_32FC1), x2(3, 1, CV_32FC1);
  static cv::Mat r1(3, 3, CV_32FC1), r2(3, 3, CV_32FC1);
  x2 = arm_trans.clone();
  r2 = arm_rot.clone();

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
      detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00008f); // 検出器（自分で設定）
      descriptorExtractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00005f);
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

  static int loop_count = 0; loop_count++;
  if(loop_count < 3){
      dst1 = dst2.clone();
      keypoints1 = keypoints2;
      descriptor1 = descriptor2.clone();    
      mask1 = mask2.clone();
      x1 = x2.clone();
      r1 = r2.clone();
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
    int mask_count = 0;
    if(!mask1.empty() && !mask2.empty()){
      cv::Point2d pt1, pt2;
      for(size_t i = 0; i < keypoints1.size(); i++){
        for(size_t j = 0; j < keypoints2.size(); j++){
          pt1 = keypoints1[i].pt;
          pt2 = keypoints2[j].pt;
          if(mask1.at<uchar>(pt1) == 255 && mask2.at<uchar>(pt2) == 255){ //キーポイント点ではどちらもマスクが白なら
            mask12.at<uchar>(i, j) = 255; //その点についてはマスクを除去してあげよう
            mask_count++;
          }
        }
      }
    }
    printf("count num = %d\n",mask_count);
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

  ////    matching 終了   ////

  ////    reconstruction 開始     ////
  //カメラの内部パラメータ(チェッカーボードから求めた焦点距離と主点座標)
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  double fovx = 364.283456, fovy = 366.617877, u0 = 159.699646, v0 = 155.864578;
  cameraMatrix = (cv::Mat_<double>(3,3) << fovx, 0.0, u0, 0.0, fovy, v0, 0.0, 0.0, 1.0);

  //カメラ座標にて三角測量
  if(dmatch.size() > 5){
    std::vector<cv::Point2d> p1;
    std::vector<cv::Point2d> p2;

    //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
    for (size_t i = 0; i < dmatch.size(); i++) {	//特徴点の数だけ処理
      cv::Mat ip(3, 1, CV_64FC1);	//3×1のベクトル
      cv::Point2d p;

      ip.at<double>(0) = keypoints1[dmatch[i].queryIdx].pt.x;	//1枚目の画像の特徴点のx座標を取得
      ip.at<double>(1) = keypoints1[dmatch[i].queryIdx].pt.y;	//1枚目の画像の特徴点のy座標を取得
      ip.at<double>(2) = 1.0;	//同次座標(焦点距離入れたほうがいいかも？)

      ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      p.x = ip.at<double>(0);
      p.y = ip.at<double>(1);
      p1.push_back(p);

      ip.at<double>(0) = keypoints2[dmatch[i].trainIdx].pt.x;	//2枚目の画像の特徴点のx座標を取得
      ip.at<double>(1) = keypoints2[dmatch[i].trainIdx].pt.y;	//2枚目の画像の特徴点のy座標を取得
      ip.at<double>(2) = 1.0;

      ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      p.x = ip.at<double>(0);
      p.y = ip.at<double>(1);
      p2.push_back(p);
    }
    //１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat r(3, 3, CV_64FC1), t(3, 1, CV_64FC1);
    cv::Mat maskmask; //RANSACの結果を保持するためのマスク
    cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003, maskmask);	//RANSACによって
    cv::recoverPose(essentialMat, p1, p2, r, t);

    //アームの運動学で求めた位置・姿勢
    cv::Mat r_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1), r_32(3, 3, CV_32FC1), t_32(3, 1, CV_32FC1);
    static cv::Mat t_sum(3, 1, CV_32FC1), r_sum(3, 3, CV_32FC1);
    r.convertTo(r_32, CV_32FC1);
    t.convertTo(t_32, CV_32FC1);
    r_arm = r2 * r1.inv();
    t_arm = r1 * (x2 - x1);
    //足していく
    static bool sum_state= false;
    if (!sum_state){
      r_sum = r2.clone();
      t_sum = x2.clone();
      sum_state = true;
    }else{
      t_sum += t_32;
      r_sum = r_32 * r_sum;
    }
    printf("loop_count #%d\n", loop_count);
    printf("t_32  = [%0.2f %0.2f %0.2f]\n", t_32.at<float>(0), t_32.at<float>(1), t_32.at<float>(2));
    printf("t_arm = [%0.2f %0.2f %0.2f]\n", t_arm.at<float>(0), t_arm.at<float>(1), t_arm.at<float>(2));
    printf("t_sum = [%0.2f %0.2f %0.2f]\n", t_sum.at<float>(0), t_sum.at<float>(1), t_sum.at<float>(2));
    printf("r_32  = [%0.2f %0.2f %0.2f \n          %0.2f %0.2f %0.2f\n          %0.2f %0.2f %0.2f]\n", r_32.at<float>(0,0), r_32.at<float>(0,1), r_32.at<float>(0,2), r_32.at<float>(1,0), r_32.at<float>(1,1), r_32.at<float>(1,2), r_32.at<float>(2,0), r_32.at<float>(2,1), r_32.at<float>(2,2));
    printf("r1    = [%0.2f %0.2f %0.2f \n          %0.2f %0.2f %0.2f\n          %0.2f %0.2f %0.2f]\n", r1.at<float>(0,0), r1.at<float>(0,1), r1.at<float>(0,2), r1.at<float>(1,0), r1.at<float>(1,1), r1.at<float>(1,2), r1.at<float>(2,0), r1.at<float>(2,1), r1.at<float>(2,2));
    printf("r_arm = [%0.2f %0.2f %0.2f \n          %0.2f %0.2f %0.2f\n          %0.2f %0.2f %0.2f]\n", r_arm.at<float>(0,0), r_arm.at<float>(0,1), r_arm.at<float>(0,2), r_arm.at<float>(1,0), r_arm.at<float>(1,1), r_arm.at<float>(1,2), r_arm.at<float>(2,0), r_arm.at<float>(2,1), r_arm.at<float>(2,2));
    printf("r_sum = [%0.2f %0.2f %0.2f \n          %0.2f %0.2f %0.2f\n          %0.2f %0.2f %0.2f]\n", r_sum.at<float>(0,0), r_sum.at<float>(0,1), r_sum.at<float>(0,2), r_sum.at<float>(1,0), r_sum.at<float>(1,1), r_sum.at<float>(1,2), r_sum.at<float>(2,0), r_sum.at<float>(2,1), r_sum.at<float>(2,2));


    //tfで5点アルゴリズムで推定したカメラ座標系
    tf2_ros::StaticTransformBroadcaster broadcaster(node);
    geometry_msgs::msg::TransformStamped msg;
    msg.transform.translation.x = t_sum.at<float>(0);
    msg.transform.translation.y = t_sum.at<float>(1);
    msg.transform.translation.z = t_sum.at<float>(2);
    //回転行列
    float qx, qy, qz, qw; 
    transformRotMatToQuaternion(qx, qy, qz, qw,
          r_sum.at<float>(0,0), r_sum.at<float>(1,0), r_sum.at<float>(2,0),
          r_sum.at<float>(0,1), r_sum.at<float>(1,1), r_sum.at<float>(2,1),
          r_sum.at<float>(0,2), r_sum.at<float>(1,2), r_sum.at<float>(2,2)
    );
    msg.transform.rotation.x = qx;
    msg.transform.rotation.y = qy;
    msg.transform.rotation.z = qz;
    msg.transform.rotation.w = qw;
    //
    msg.header.frame_id = "map";
    msg.child_frame_id = "camera_5point";

    broadcaster.sendTransform(msg);


    //正規化座標系で計算しているのでProjection matrix = 外部カメラパラメータ行列
    cv::Mat prjMat1, prjMat2;
    prjMat1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0
    prjMat2 = cv::Mat(3, 4, CV_32FC1);  //そこからどれだけ回転、並進したか

    for (int i = 0; i < 3; i++) {
      //prjMat2.at<float>(i, 3) = t.at<float>(i);
      prjMat2.at<float>(i, 3) = t_arm.at<float>(i);
      for (int j = 0; j < 3; j++) {
        //prjMat2.at<float>(i, j) = r.at<float>(i, j);
        prjMat2.at<float>(i, j) = r_arm.at<float>(i, j);
      }
    }
    
    cv::Mat point4D;
    cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point4D);	//三角測量
    //ここまで
    
    // viz用に並べ替え&距離が長すぎるのは削除
    cv::Mat pointCloud(point4D.cols, 1, CV_32FC3);
    cv::Mat pointCloud2(point4D.cols, 1, CV_32FC3);
    for (int i = 0; i < point4D.cols; i++) {
      for (int j = 0; j <  3; j++) {
          pointCloud.at<cv::Vec3f>(i)[j] = point4D.at<float>(j, i);
      }
    }
    float dist = 0.0;
    int dist_count = 0;
    for (int i = 0; i < point4D.cols; i++){
      if(pointCloud.at<cv::Vec3f>(i)[2] < 10.0 && pointCloud.at<cv::Vec3f>(i)[2] > -10.0){
        for(int j = 0; j < 3; j++){
          pointCloud2.at<cv::Vec3f>(i - dist_count)[j] = pointCloud.at<cv::Vec3f>(i)[j];
        }
      } else {
        dist_count++;
      }
      dist += pointCloud2.at<cv::Vec3f>(i - dist_count)[2];
    }
    dist = dist / (point4D.cols - dist_count);
    printf("average_dist = %0.2f, cols,rows = [%d, %d]\n", dist, point4D.cols, pointCloud2.rows);
    
    // 点群の描画
    if(show_camera){
      cv::viz::WCloud cloud(pointCloud2);
      visualizeWindow.showWidget("CLOUD", cloud);
      visualizeWindow.spinOnce(1, true);
      //visualizeWindow.spin();
    }
    
    ////    reconstruction 終了     ////

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_pub_pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

    convert_frame_to_message(cvframe, atoi(msg_image->header.frame_id.c_str()), *msg_pub);  //cv → msg
    //convert_point3D_to_PCL(point4D, atoi(msg_pub_pointcloud->header.frame_id.c_str()), *msg_pub_pointcloud);
    pub->publish(std::move(msg_pub)); 
    //pub_pointcloud->publish(std::move(msg_pub_pointcloud));
  }

  // 1つめのフレームに代入
  dst1 = dst2.clone();
  keypoints1 = keypoints2;
  descriptor1 = descriptor2.clone(); 
  mask1 = mask2.clone();  
  x1 = x2.clone();
  r1 = r2.clone();

}

int main(int argc, char * argv[])
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
    bool mask = false;

    std::string topic_sub("endoscope_image");   
    std::string topic_sub_mask("mask_image");   
    std::string topic_sub_arm("arm_trans");
    std::string topic_pub("feature_point");
    std::string topic_pub_pointcloud("pointcloud");
    
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
    }
    //viz用
    cv::viz::Viz3d visualizeWindow("3Dview");
    cv::waitKey(1);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("reconstruction");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());

    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos); // Create the image publisher with our custom QoS profile.
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    //グローバル座標とカメラ座標間の座標変換行列
    cv::Mat arm_trans(3, 1, CV_32FC1);
    cv::Mat arm_rot(3, 3, CV_32FC1);
    auto callback_arm_trans = [&arm_trans, &arm_rot, &node](const geometry_msgs::msg::Transform::SharedPtr msg_sub){
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

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
    message_filters::Subscriber<sensor_msgs::msg::Image> mask_image_sub(node.get(), topic_sub_mask);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(image_sub, mask_image_sub, 10);
    auto sub_arm = node->create_subscription<geometry_msgs::msg::Transform>(topic_sub_arm, qos, callback_arm_trans);  
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, mask, visualizeWindow, arm_trans, arm_rot, node_logger, pub, pub_pointcloud, node));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}