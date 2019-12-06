#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ktl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../include/options_reconstruction.hpp"

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

void convert_pointcloud_to_PCL(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 & msg_cloud_pub, int dist_count){
  msg_cloud_pub.header = std_msgs::msg::Header();
  msg_cloud_pub.header.stamp = rclcpp::Clock().now();
  msg_cloud_pub.header.frame_id = "map";
  
  msg_cloud_pub.is_bigendian = false;
  msg_cloud_pub.is_dense = true;

  msg_cloud_pub.height = 1;
  msg_cloud_pub.width = pointCloud2.rows;

  msg_cloud_pub.fields.resize(3);
  msg_cloud_pub.fields[0].name = "x";
  msg_cloud_pub.fields[1].name = "y";
  msg_cloud_pub.fields[2].name = "z";

  sensor_msgs::msg::PointField::_offset_type offset = 0;
  for (uint32_t i = 0; i < msg_cloud_pub.fields.size(); ++i, offset += sizeof(float)) {
    msg_cloud_pub.fields[i].count = 1;
    msg_cloud_pub.fields[i].offset = offset;
    msg_cloud_pub.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  msg_cloud_pub.point_step = offset;
  msg_cloud_pub.row_step = msg_cloud_pub.point_step * msg_cloud_pub.width;
  msg_cloud_pub.data.resize(msg_cloud_pub.row_step * msg_cloud_pub.height);

  auto floatData = reinterpret_cast<float *>(msg_cloud_pub.data.data());
  for (uint32_t i = 0; i < msg_cloud_pub.width - dist_count; ++i) {
    for (uint32_t j = 0; j < 3; ++j){
      floatData[i * (msg_cloud_pub.point_step / sizeof(float)) + j] = pointCloud2.at<cv::Vec3f>(i)[j];
    }
  } 
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

void display_tf(cv::Mat t_sum, cv::Mat r_sum, std::shared_ptr<rclcpp::Node> node
){
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped msg;
  float qx, qy, qz, qw; 
    transformRotMatToQuaternion(qx, qy, qz, qw,
          r_sum.at<float>(0,0), r_sum.at<float>(1,0), r_sum.at<float>(2,0),
          r_sum.at<float>(0,1), r_sum.at<float>(1,1), r_sum.at<float>(2,1),
          r_sum.at<float>(0,2), r_sum.at<float>(1,2), r_sum.at<float>(2,2)
    );
    msg.transform.translation.x = t_sum.at<float>(0);
    msg.transform.translation.y = t_sum.at<float>(1);
    msg.transform.translation.z = t_sum.at<float>(2);
    msg.transform.rotation.x = qx;
    msg.transform.rotation.y = qy;
    msg.transform.rotation.z = qz;
    msg.transform.rotation.w = qw;
    msg.header.frame_id = "map";
    msg.child_frame_id = "camera_5point";
    broadcaster.sendTransform(msg);
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> & msg_image, const std::shared_ptr<const sensor_msgs::msg::Image> & msg_mask
  , bool show_camera, size_t feature, size_t match, bool mask, cv::Mat arm_trans, cv::Mat arm_rot, size_t prjMat, rclcpp::Logger logger 
  , std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud, std::shared_ptr<rclcpp::Node> node)
  {
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  
  //frame間隔をあける
  int frame_span;
  static int frame1 = 0, frame2;
  frame2 = atoi(msg_image->header.frame_id.c_str());
  frame_span = frame2 - frame1;
  if(frame2 < 10 || frame_span < 10){
    return;
  }

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
      detector = cv::ORB::create(1000, 1.2f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
      descriptorExtractor = cv::ORB::create(1000, 1.2f, 30, 31, 0 , 2, cv::ORB::HARRIS_SCORE, 31, 5);
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
      frame1 = frame2;
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
    //画像間でのカメラの移動量（運動学より）
    cv::Mat r_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
    r_arm = r2 * r1.inv();
    t_arm = r1 * (x2 - x1);
    cv::Point2f p1 = cv::Point2f(msg_image->height / 2., msg_image->width / 2.);
    cv::Point2f center = cv::Point2f(t_arm.at<float>(0) * 10 + p1.x, t_arm.at<float>(1) * 10 + p1.y);
    cv::Scalar color = cv::Scalar(0, 255, 0);
    cv::arrowedLine(cvframe, p1, center, color, 2, 8, 0, 0.5);
    //表示
    cv::imshow("cvframe", cvframe);
    cv::waitKey(1);
  }

  ////    matching 終了   ////

  ////    reconstruction 開始     ////

  if(dmatch.size() > 5){  //５点アルゴリズムが行えるのに十分対応点があれば
    //カメラの内部パラメータ(チェッカーボードから求めた焦点距離と主点座標)
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    double fovx = 364.283456, fovy = 366.617877, u0 = 159.699646, v0 = 155.864578;
    cameraMatrix = (cv::Mat_<double>(3,3) << fovx, 0.0, u0, 0.0, fovy, v0, 0.0, 0.0, 1.0);

    std::vector<cv::Point2d> p1;
    cv::Mat p1_cv(1, dmatch.size(), CV_64FC2);
    std::vector<cv::Point2d> p2;
    cv::Mat p2_cv(1, dmatch.size(), CV_64FC2);

    //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
    for (size_t i = 0; i < dmatch.size(); i++) {	//特徴点の数だけ処理
      cv::Mat ip(3, 1, CV_64FC1);	//3×1のベクトル
      cv::Point2d p;

      //ip.at<double>(0) = keypoints1[dmatch[i].queryIdx].pt.x;	//1枚目の画像の特徴点のx座標を取得
      //ip.at<double>(1) = keypoints1[dmatch[i].queryIdx].pt.y;	//1枚目の画像の特徴点のy座標を取得
      //ip.at<double>(2) = 1.0;	//同次座標用

      //ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      //p.x = ip.at<double>(0);
      //p.y = ip.at<double>(1);
      p.x = keypoints1[dmatch[i].queryIdx].pt.x;
      p.y = keypoints1[dmatch[i].queryIdx].pt.y;
      p1.push_back(p);

      //ip.at<double>(0) = keypoints2[dmatch[i].trainIdx].pt.x;	//2枚目の画像の特徴点のx座標を取得
      //ip.at<double>(1) = keypoints2[dmatch[i].trainIdx].pt.y;	//2枚目の画像の特徴点のy座標を取得
      //ip.at<double>(2) = 1.0;

      //ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      //p.x = ip.at<double>(0);
      //p.y = ip.at<double>(1);
      p.x = keypoints2[dmatch[i].queryIdx].pt.x;
      p.y = keypoints2[dmatch[i].queryIdx].pt.y;
      p2.push_back(p);
    }
    //１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat r(3, 3, CV_64FC1), t(3, 1, CV_64FC1);
    cv::Mat maskmask; //RANSACの結果を保持するためのマスク
    cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003, maskmask);	//RANSACによって
    cv::recoverPose(essentialMat, p1, p2, r, t);
    //printf("t = [%lf %lf %lf]\n", t.at<double>(0), t.at<double>(1), t.at<double>(2));

    //画像間でのカメラの移動量（運動学より）
    cv::Mat r_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
    r_arm = r2 * r1.inv();
    t_arm = r1 * (x2 - x1);
    //printf("t_arm = [%0.2f %0.2f %0.2f]\n", t_arm.at<float>(0), t_arm.at<float>(1), t_arm.at<float>(2));

    //5点アルゴリズムで求めた座標をtfで表示するために移動量を累積する
    cv::Mat r_32(3, 3, CV_32FC1), t_32(3, 1, CV_32FC1);
    static cv::Mat t_sum(3, 1, CV_32FC1), r_sum(3, 3, CV_32FC1);
    r.convertTo(r_32, CV_32FC1);
    t.convertTo(t_32, CV_32FC1);
    static bool sum_state= false;
    if (!sum_state){
      r_sum = r2.clone();
      t_sum = x2.clone();
      sum_state = true;
    }else{
      t_sum += t_32;
      r_sum = r_32 * r_sum;
    }
    ////カメラの移動量をx軸と水平方向の移動のみ
    //１つめの画像の方向を

    //三角測量のためのカメラの透視射影行列
     cv::Mat prjMat1(3, 4, CV_64FC1), prjMat2(3, 4, CV_64FC1), Rt1(3, 4, CV_64FC1), Rt2(3, 4, CV_64FC1);
    if(prjMat == 0){//５点アルゴリズムで推定したカメラの位置姿勢から三角測量をする（カメラ空間）
      Rt1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          Rt2.at<double>(i, j) = r.at<double>(i, j);
        }
        Rt2.at<double>(i, 3) = t.at<double>(i);
      }
      prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
      prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列
    } else if(prjMat == 1){//運動学から求めたカメラの位置姿勢から三角測量をする（カメラ空間）
      Rt1 = cv::Mat::eye(3, 4, CV_64FC1); //片方は回転、並進ともに0の外部パラメータ
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          Rt2.at<double>(i, j) = r_arm.at<float>(i, j)*1000;
        }
        Rt2.at<double>(i, 3) = t_arm.at<float>(i)*1000;
      }
      prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
      prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列
    } else if(prjMat == 2){//運動学から求めたカメラの位置姿勢から三角測量をする（ワールド座標）
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          Rt1.at<double>(i, j) = r1.at<float>(i, j);
          Rt2.at<double>(i, j) = r2.at<float>(i, j);
        }
        Rt1.at<double>(i, 3) = x1.at<float>(i);
        Rt2.at<double>(i, 3) = x2.at<float>(i);
      }
      prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
      prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列
    }
    
    //三角測量（triangulatePoints()に一気に点を与えるとき）
    cv::Mat point4D;
    cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point4D);
    //prjMat1 : 運動学で求めたカメラ空間上での動作前のカメラの位置・姿勢行列
    //prjMat2 : 運動学で求めたカメラ空間上での動作後のカメラの位置・姿勢行列
    //p1      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間にで表現したもの
    //p2      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間で表現したもの
    //point4D : 三角測量で求めた各特徴点座標をカメラ空間にて表現したもの
    
    //Homogeneus座標からEuclid座標系への変換
    cv::Mat point3D;
    cv::convertPointsFromHomogeneous(point4D.t(), point3D);
    
    // RViz2用に並べ替えmatcmatc
    cv::Mat pointCloud(point3D.rows, 1, CV_32FC3);
    cv::Mat pointCloud2(point3D.rows, 1, CV_32FC3);
    for (int i = 0; i < point3D.rows; i++) {
      for (int j = 0; j <  3; j++) {
          pointCloud.at<cv::Vec3f>(i)[j] = point3D.at<cv::Vec3d>(i)[j];
      }
    }
    //printf("dmatch: %zu\n", dmatch.size());
    //printf("point4D: %d %d %d\n", point4D.rows, point4D.cols, point4D.channels());
    //printf("point3D: %d %d %d\n", point3D.rows, point3D.cols, point3D.channels());
    
    for (int i = 0; i < point3D.rows; i++) {
      //printf("point4D #%d =[%0.4lf %0.4lf %0.4lf %0.4lf]\n", i, point4D.reshape(4, 1).at<cv::Vec3d>(i)[0], point4D.reshape(4, 1).at<cv::Vec3d>(i)[1], point4D.reshape(4, 1).at<cv::Vec3d>(i)[2], point4D.reshape(4, 1).at<cv::Vec3d>(i)[3]);
      //printf("point3D #%d =[%0.4lf %0.4lf %0.4lf]\n", i, point3D.at<cv::Vec3d>(i)[0], point3D.at<cv::Vec3d>(i)[1], point3D.at<cv::Vec3d>(i)[2]);
    }
    //距離（ノルム）が大きすぎる値は除去
    int dist_count = 0;
    for (int i = 0; i < point3D.rows; i++){
      if(pointCloud.at<cv::Vec3f>(i)[2] < 0){
        for(int j = 0; j < 3; j++){
          pointCloud2.at<cv::Vec3f>(i - dist_count)[j] = pointCloud.at<cv::Vec3f>(i)[j];
          //pointCloud2.at<cv::Vec3f>(i - dist_count)[j] = (r * pointCloud).at<cv::Vec3f>(i)[j] + x1.at<float>(j);
        }
      } else {
        dist_count++;
      }
      //printf("p(%d) = [%0.2f %0.2f %0.2f]\n", i, pointCloud2.at<cv::Vec3f>(i - dist_count)[0], pointCloud2.at<cv::Vec3f>(i - dist_count)[1], pointCloud2.at<cv::Vec3f>(i - dist_count)[2]);
    }
    ////    reconstruction 終了     ////

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_cloud_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    convert_frame_to_message(cvframe, atoi(msg_image->header.frame_id.c_str()), *msg_pub);  //cv → msg
    //convert_pointcloud_to_PCL(pointCloud2, *msg_cloud_pub, dist_count);
    convert_pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);

    pub->publish(std::move(msg_pub)); 
    pub_pointcloud->publish(*msg_cloud_pub);
    display_tf(t_sum, r_sum, node); //tfで5点アルゴリズムで推定したカメラff座標をtfに変換およびパブリッシュ
  }

  // 1つめのフレームに代入
  dst1 = dst2.clone();
  keypoints1 = keypoints2;
  descriptor1 = descriptor2.clone(); 
  mask1 = mask2.clone();  
  x1 = x2.clone();
  r1 = r2.clone();
  frame1 = frame2;
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
    size_t prjMat = 1;

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
        argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &feature, &match, &mask, &prjMat))
    {
        return 0;
    }

    if (show_camera) {
      // Initialize an OpenCV named window called "cvframe".
      cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
    }
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
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, mask, arm_trans, arm_rot, prjMat, node_logger, pub, pub_pointcloud, node));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}