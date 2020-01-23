#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2/buffer_core.h>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../include/options_reconstruction_online_tf2.hpp"

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
void convert_frame_to_message(const cv::Mat &frame, size_t frame_id, sensor_msgs::msg::Image &msg)
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

void convert_pointcloud_to_PCL(const cv::Mat pointCloud2, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count)
{
  msg_cloud_pub.header = std_msgs::msg::Header();
  msg_cloud_pub.header.stamp = rclcpp::Clock().now();
  msg_cloud_pub.header.frame_id = "world";

  msg_cloud_pub.is_bigendian = false;
  msg_cloud_pub.is_dense = true;

  msg_cloud_pub.height = 1;
  msg_cloud_pub.width = pointCloud2.rows;

  msg_cloud_pub.fields.resize(3);
  msg_cloud_pub.fields[0].name = "x";
  msg_cloud_pub.fields[1].name = "y";
  msg_cloud_pub.fields[2].name = "z";

  sensor_msgs::msg::PointField::_offset_type offset = 0;
  for (uint32_t i = 0; i < msg_cloud_pub.fields.size(); ++i, offset += sizeof(float))
  {
    msg_cloud_pub.fields[i].count = 1;
    msg_cloud_pub.fields[i].offset = offset;
    msg_cloud_pub.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  msg_cloud_pub.point_step = offset;
  msg_cloud_pub.row_step = msg_cloud_pub.point_step * msg_cloud_pub.width;
  msg_cloud_pub.data.resize(msg_cloud_pub.row_step * msg_cloud_pub.height);

  auto floatData = reinterpret_cast<float *>(msg_cloud_pub.data.data());
  for (uint32_t i = 0; i < msg_cloud_pub.width - dist_count; ++i)
  {
    for (uint32_t j = 0; j < 3; ++j)
    {
      floatData[i * (msg_cloud_pub.point_step / sizeof(float)) + j] = pointCloud2.at<cv::Vec3f>(i)[j];
    }
  }
}

void transformQuaternionToRotMat(
    float &m11, float &m12, float &m13,
    float &m21, float &m22, float &m23,
    float &m31, float &m32, float &m33,
    float qx, float qy, float qz, float qw)
{
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
    float m31, float m32, float m33)
{
  // 最大成分を検索
  float elem[4]; // 0:x, 1:y, 2:z, 3:w
  elem[0] = m11 - m22 - m33 + 1.0f;
  elem[1] = -m11 + m22 - m33 + 1.0f;
  elem[2] = -m11 - m22 + m33 + 1.0f;
  elem[3] = m11 + m22 + m33 + 1.0f;

  unsigned biggestIndex = 0;
  for (int i = 1; i < 4; i++)
  {
    if (elem[i] > elem[biggestIndex])
      biggestIndex = i;
  }

  if (elem[biggestIndex] < 0.0f)
    return false; // 引数の行列に間違いあり！

  // 最大要素の値を算出
  float *q[4] = {&qx, &qy, &qz, &qw};
  float v = sqrtf(elem[biggestIndex]) * 0.5f;
  *q[biggestIndex] = v;
  float mult = 0.25f / v;

  switch (biggestIndex)
  {
  case 0: // x
    *q[1] = (m12 + m21) * mult;
    *q[2] = (m31 + m13) * mult;
    *q[3] = (m32 - m23) * mult;
    break;
  case 1: // y
    *q[0] = (m12 + m21) * mult;
    *q[2] = (m23 + m32) * mult;
    *q[3] = (m13 - m31) * mult;
    break;
  case 2: // z
    *q[0] = (m31 + m13) * mult;
    *q[1] = (m23 + m32) * mult;
    *q[3] = (m21 - m12) * mult;
    break;
  case 3: // w
    *q[0] = (m32 - m23) * mult;
    *q[1] = (m13 - m31) * mult;
    *q[2] = (m21 - m12) * mult;
    break;
  }
  return true;
}
void QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double &roll, double &pitch, double &yaw)
{
  double q0q0 = q0 * q0;
  double q0q1 = q0 * q1;
  double q0q2 = q0 * q2;
  double q0q3 = q0 * q3;
  double q1q1 = q1 * q1;
  double q1q2 = q1 * q2;
  double q1q3 = q1 * q3;
  double q2q2 = q2 * q2;
  double q2q3 = q2 * q3;
  double q3q3 = q3 * q3;
  roll = atan2(2.0 * (q2q3 + q0q1), q0q0 - q1q1 - q2q2 + q3q3);
  pitch = asin(2.0 * (q0q2 - q1q3));
  yaw = atan2(2.0 * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3);
}

float revdeg(cv::Mat R_arm)
{
  //回転行列をクォータニオンに変換
  float qx, qy, qz, qw;
  transformRotMatToQuaternion(qx, qy, qz, qw,
                              R_arm.at<float>(0, 0), R_arm.at<float>(0, 1), R_arm.at<float>(0, 2),
                              R_arm.at<float>(1, 0), R_arm.at<float>(1, 1), R_arm.at<float>(1, 2),
                              R_arm.at<float>(2, 0), R_arm.at<float>(2, 1), R_arm.at<float>(2, 2));
  //クォータニオンの4つめの要素から回転角を取り出す
  float phi = 2 * std::acos(qw);
  return phi;
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image, const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
              bool show_camera, size_t feature, size_t match, size_t prjMat, rclcpp::Logger logger,
              std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud,
              tf2_ros::Buffer *buffer_)
{
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  //ループカウンタ
  static int loop_count = 0;
  loop_count++;

  //グローバル座標とカメラ座標間の座標変換行列
  cv::Mat arm_trans(3, 1, CV_32FC1);
  cv::Mat arm_rot(3, 3, CV_32FC1);
  // //tfでの座標の受取
  geometry_msgs::msg::TransformStamped TransformStamped;
  TransformStamped = buffer_->lookupTransform("world", "endoscope", tf2::TimePoint());
  //並進成分
  arm_trans.at<float>(0) = (float)TransformStamped.transform.translation.x * 1000.;
  arm_trans.at<float>(1) = (float)TransformStamped.transform.translation.y * 1000.;
  arm_trans.at<float>(2) = (float)TransformStamped.transform.translation.z * 1000.;
  //回転成分
  transformQuaternionToRotMat(arm_rot.at<float>(0, 0), arm_rot.at<float>(0, 1), arm_rot.at<float>(0, 2),
                              arm_rot.at<float>(1, 0), arm_rot.at<float>(1, 1), arm_rot.at<float>(1, 2),
                              arm_rot.at<float>(2, 0), arm_rot.at<float>(2, 1), arm_rot.at<float>(2, 2),
                              (float)TransformStamped.transform.rotation.x, (float)TransformStamped.transform.rotation.y, (float)TransformStamped.transform.rotation.z, (float)TransformStamped.transform.rotation.w);

  double rall, pitch, yaw;
  QuaternionToEulerAngles(TransformStamped.transform.rotation.x, TransformStamped.transform.rotation.y, TransformStamped.transform.rotation.z, TransformStamped.transform.rotation.w, rall, pitch, yaw);
  printf("position = [%0.4f %0.4f %0.4f] [%0.4f %0.4f %0.4f]\n", arm_trans.at<float>(0), arm_trans.at<float>(1), arm_trans.at<float>(2), rall, pitch, yaw);
  
  //frame間隔
  static int frame_key1, frame_key2, frame_num;
  frame_num = atoi(msg_image->header.frame_id.c_str());

  //Subscribe
  cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
  if (msg_image->encoding == "rgb8")
  {
    cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
  }

  //アーム姿勢をcv::Mat形式に変換
  static cv::Mat R_keyframe(3, 3, CV_32FC1), R_keyframe1(3, 3, CV_32FC1), R_keyframe2(3, 3, CV_32FC1), R_frame(3, 3, CV_32FC1);
  static cv::Mat t_keyframe(3, 1, CV_32FC1), t_keyframe1(3, 1, CV_32FC1), t_keyframe2(3, 1, CV_32FC1), t_frame(3, 1, CV_32FC1);
  R_frame = arm_rot.clone();
  t_frame = arm_trans.clone();

  ////  detection 開始  ////
  static cv::Mat dst_keyframe, dst_keyframe1, dst_keyframe2, descriptor_keyframe1, descriptor_keyframe2;
  static std::vector<cv::KeyPoint> keypoints_keyframe, keypoints_keyframe1, keypoints_keyframe2;
  cv::Mat dst_frame, descriptor_frame;
  std::vector<cv::KeyPoint> keypoints_frame;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

  dst_frame = frame_image.clone();
  if (feature == 0)
  {
    // AKAZE特徴抽出
    detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.000008f); // 検出器（自分で設定）
    descriptorExtractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.000008f);
  }
  else if (feature == 1)
  {
    //ORB特徴量抽出
    detector = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
    descriptorExtractor = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
  }
  else if (feature == 2)
  {
    //BRISK特徴量抽出
    detector = cv::BRISK::create(120, 3, 0.6f);
    descriptorExtractor = cv::BRISK::create();
  }
  else
  {
    printf("Choosing Incorrect Option of Feature point detector.\n");
    return;
  }
  detector->detect(dst_frame, keypoints_frame);
  descriptorExtractor->compute(dst_frame, keypoints_frame, descriptor_frame);

  //特徴抽出初めてやったときはdst_keyframe1になんにも入ってない
  static bool set_bool1 = false, set_bool2 = false;
  if (!set_bool1)
  {
    if (set_bool2)
    {
      dst_keyframe1 = dst_keyframe2.clone();
      keypoints_keyframe1 = keypoints_keyframe2;
      descriptor_keyframe1 = descriptor_keyframe2.clone();
      R_keyframe1 = R_keyframe2.clone();
      t_keyframe1 = t_keyframe2.clone();
      frame_key1 = frame_key2;
      set_bool1 = true;
    }
    dst_keyframe2 = dst_frame.clone();
    keypoints_keyframe2 = keypoints_frame;
    descriptor_keyframe2 = descriptor_frame.clone();
    R_keyframe2 = R_frame.clone();
    t_keyframe2 = t_frame.clone();
    frame_key2 = frame_num;
    set_bool2 = true;
    return;
  }
  ////    detection 終了  ////

  ////    matching 開始   ////
  //対応点の探索
  cv::Ptr<cv::DescriptorMatcher> matcher;
  if (match == 0)
  {
    //Brute-Force matcher
    matcher = cv::DescriptorMatcher::create("BruteForce");
  }
  else if (match == 1)
  {
    //FLANN
    matcher = cv::DescriptorMatcher::create("FlannBased");
  }
  else
  {
    printf("Choosing Incorrect Option of Matcher\n");
    return;
  }

  //マッチング
  std::vector<cv::DMatch> dmatch, good_dmatch;
  std::vector<cv::DMatch> dmatch12, dmatch21;
  std::vector<cv::Point2f> match_point1, match_point2;

  // KeyFrame1とKeyframe2のどちらを使うか決定
  cv::Mat R_arm(3, 3, CV_32FC1), R_arm1(3, 3, CV_32FC1), R_arm2(3, 3, CV_32FC1), R_endo(3, 3, CV_32FC1);
  cv::Mat t_arm(3, 1, CV_32FC1), t_arm1(3, 1, CV_32FC1), t_arm2(3, 1, CV_32FC1), t_endo(3, 1, CV_32FC1);
  R_arm1 = R_frame * R_keyframe1.t(); //カメラの回転変化
  R_arm2 = R_frame * R_keyframe2.t(); //カメラの回転変化
  t_arm1 = t_frame - t_keyframe1;     //ワールド座標系でのカメラ移動量
  t_arm2 = t_frame - t_keyframe2;     //ワールド座標系でのカメラ移動量
  float phi1 = revdeg(R_arm1);
  float phi2 = revdeg(R_arm2);
  float Keyframedetect = std::abs(phi1) - std::abs(phi2);
  if (Keyframedetect > 0)
  {
    R_arm = R_arm1.clone();
    t_arm = t_arm1.clone();
    R_keyframe = R_keyframe1.clone();
    t_keyframe = t_keyframe1.clone();
    dst_keyframe = dst_keyframe1;
    matcher->match(descriptor_keyframe1, descriptor_frame, dmatch12); //dst_keyframe1 -> dst_frame
    matcher->match(descriptor_frame, descriptor_keyframe1, dmatch21); //dst_frame -> dst_keyframe1
    keypoints_keyframe = keypoints_keyframe1;
  }
  else
  {
    R_arm = R_arm2.clone();
    t_arm = t_arm2.clone();
    R_keyframe = R_keyframe2.clone();
    t_keyframe = t_keyframe2.clone();
    dst_keyframe = dst_keyframe2;
    matcher->match(descriptor_keyframe2, descriptor_frame, dmatch12); //dst_keyframe1 -> dst_frame
    matcher->match(descriptor_frame, descriptor_keyframe2, dmatch21); //dst_frame -> dst_keyframe1
    keypoints_keyframe = keypoints_keyframe2;
  }

  //エンコーダでの変化量を計算
  R_endo = R_arm.t();
  t_endo = R_keyframe * t_arm; //(１つ目の画像撮影時の)カメラ座標系に合わせたカメラ移動量

  //dst_keyframe1 -> dst_frame と dst_frame -> dst_keyframe1の結果が一致しているか検証
  size_t good_count = 0;
  for (size_t i = 0; i < dmatch12.size(); ++i)
  {
    cv::DMatch m12 = dmatch12[i];
    cv::DMatch m21 = dmatch21[m12.trainIdx];

    if (m21.trainIdx == m12.queryIdx)
    { // 一致しているものだけを抜粋
      dmatch.push_back(m12);
      match_point1.push_back(keypoints_keyframe[dmatch12[i].queryIdx].pt);
      match_point2.push_back(keypoints_frame[dmatch12[i].trainIdx].pt);
      good_count++;
    }
  }
  //ホモグラフィ行列推定
  const size_t MIN_MATCH_COUNT = 10;
  if (good_count > MIN_MATCH_COUNT)
  { //十分対応点が見つかるならば
    cv::Mat masks;
    cv::Mat H = cv::findHomography(match_point1, match_point2, masks, cv::RANSAC, 5.);
    //RANSACで使われた対応点のみ抽出
    for (int i = 0; i < masks.rows; ++i)
    {
      uchar *inliner = masks.ptr<uchar>(i);
      if (inliner[0] == 1)
      {
        good_dmatch.push_back(dmatch[i]);
      }
    }
  }
  ////    matching 終了   ////

  ////    reconstruction 開始     ////
  size_t match_num = good_dmatch.size();
  if (match_num > 5) //５点アルゴリズムが行えるのに十分対応点があれば
  {
    //カメラの内部パラメータ(チェッカーボードから求めた焦点距離と主点座標)
    cv::Mat cameraMatrix(3, 3, CV_32FC1), cameraMatrix_64(3, 3, CV_64FC1);
    float fovx = 396.7, fovy = 396.9, u0 = 163.6, v0 = 157.1;
    cameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0,
                    0.0, fovy, v0,
                    0.0, 0.0, 1.0);
    cameraMatrix.convertTo(cameraMatrix_64, CV_64FC1);

    //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
    std::vector<cv::Point2f> p1, p2;
    for (size_t i = 0; i < match_num; i++)
    { //特徴点の数だけ処理
      cv::Point2f p_1, p_2;
      p_1.x = keypoints_keyframe[good_dmatch[i].queryIdx].pt.x;
      p_1.y = keypoints_keyframe[good_dmatch[i].queryIdx].pt.y;
      p_2.x = keypoints_frame[good_dmatch[i].trainIdx].pt.x;
      p_2.y = keypoints_frame[good_dmatch[i].trainIdx].pt.y;
      p1.push_back(p_1);
      p2.push_back(p_2);
    }

    //三角測量のためのカメラの透視射影行列
    cv::Mat prjMat1(3, 4, CV_32FC1), prjMat2(3, 4, CV_32FC1);
    cv::Mat Rt1(3, 4, CV_32FC1), Rt2(3, 4, CV_32FC1);
    cv::Mat r_32(3, 3, CV_32FC1), t_32(3, 1, CV_32FC1);

    if (prjMat == 0)
    {
      //5点アルゴリズム
      //１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
      cv::Mat r(3, 3, CV_64FC1), t(3, 1, CV_64FC1);
      cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003); //RANSACによって
      cv::recoverPose(essentialMat, p1, p2, r, t);
      r.convertTo(r_32, CV_32FC1);
      t.convertTo(t_32, CV_32FC1);
      Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0
      cv::hconcat(r_32, t_32, Rt2);
    }
    else if (prjMat == 1)
    {                                     //運動学から求めたカメラの位置姿勢から三角測量をする（カメラ空間）
      Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0の外部パラメータ
      cv::hconcat(R_endo.t(), -t_endo, Rt2);
    }
    prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
    prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列

    //三角測量（triangulatePoints()に一気に点を与えるとき）
    cv::Mat point3D, point3D_arm;
    cv::Mat point4D(4, match_num, CV_32FC1);
    float dist_mean;
    for (size_t i = 0; i < match_num; ++i)
    {
      // std::vector<cv::Point3f> point3D_result;
      cv::Mat point3D_result, point3D_result_arm;
      cv::triangulatePoints(prjMat1, prjMat2, cv::Mat(p1[i]), cv::Mat(p2[i]), point4D);
      cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
      point3D_result_arm = (R_keyframe * point3D_result.reshape(1, 3) + t_keyframe) / 1000.;
      point3D.push_back(point3D_result);
      point3D_arm.push_back(point3D_result_arm.reshape(3, 1));
      dist_mean += point3D_result.at<cv::Vec3f>(0, 0)[2];
    }
    dist_mean = dist_mean / match_num;

    // RViz2用に並べ替え
    cv::Mat pointCloud(match_num, 1, CV_32FC3), pointCloud_arm(match_num, 1, CV_32FC3);
    point3D.convertTo(pointCloud, CV_32FC3);
    point3D_arm.convertTo(pointCloud_arm, CV_32FC3);

    //cv::imshow
    cv::Mat cvframe, cvframe_, cvframe_t;
    cv::drawMatches(dst_keyframe, keypoints_keyframe, dst_frame, keypoints_frame, dmatch, cvframe);
    cv::drawMatches(dst_keyframe, keypoints_keyframe, dst_frame, keypoints_frame, good_dmatch, cvframe_);
    if (show_camera)
    {
      cv::Point2f center_t, center_t_arm;
      cv::Point2f p_center = cv::Point2f(msg_image->height / 2., msg_image->width / 2.);
      cv::Scalar color = cv::Scalar(0, 255, 0);
      center_t_arm = cv::Point2f(t_endo.at<float>(0) * 20 + p_center.x, t_endo.at<float>(1) * 20 + p_center.y);
      if (prjMat == 0)
      {
        center_t = cv::Point2f(t_32.at<float>(0, 0) * 20 + p_center.x, t_32.at<float>(1, 0) * 20 + p_center.y);
        cvframe_t = cvframe_.clone();
        cv::arrowedLine(cvframe, p_center, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_, p_center, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_t, p_center, center_t, color, 2, 8, 0, 0.5);
        cv::imshow("cvframe", cvframe);
        cv::imshow("cvframe_", cvframe_);
        cv::imshow("cvframe_t", cvframe_t);
      }
      else if (prjMat == 1)
      {
        cv::arrowedLine(cvframe, p_center, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_, p_center, center_t_arm, color, 2, 8, 0, 0.5);
        cv::imshow("cvframe", cvframe);
        cv::imshow("cvframe_", cvframe_);
      }
      cv::waitKey(3);
    }

    //変数のサイズ確認
    // printf("dmatch: %zu\n", match_num);
    // printf("t_keyframe1   : %d %d %d\n", t_keyframe1.rows, t_keyframe1.cols, t_keyframe1.channels());
    // printf("Rt1           : %d %d %d\n", Rt1.rows, Rt1.cols, Rt1.channels());
    // printf("point4D       : %d %d %d\n", point4D.rows, point4D.cols, point4D.channels());
    // printf("point3D       : %d %d %d\n", point3D.rows, point3D.cols, point3D.channels());
    // printf("pointCloud    : %d %d %d\n", pointCloud.rows, pointCloud.cols, pointCloud.channels());

    // printf
    // printf("cameramatrix  = \n%0.4f %0.4f %0.4f\n%0.4f %0.4f %0.4f\n%0.4f %0.4f %0.4f\n", cameraMatrix.at<float>(0, 0), cameraMatrix.at<float>(0, 1), cameraMatrix.at<float>(0, 2), cameraMatrix.at<float>(1, 0), cameraMatrix.at<float>(1, 1), cameraMatrix.at<float>(1, 2), cameraMatrix.at<float>(2, 0), cameraMatrix.at<float>(2, 1), cameraMatrix.at<float>(2, 2));
    // printf("Rt1           = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", Rt1.at<float>(0, 0), Rt1.at<float>(0, 1), Rt1.at<float>(0, 2), Rt1.at<float>(0, 3), Rt1.at<float>(1, 0), Rt1.at<float>(1, 1), Rt1.at<float>(1, 2), Rt1.at<float>(1, 3), Rt1.at<float>(2, 0), Rt1.at<float>(2, 1), Rt1.at<float>(2, 2), Rt1.at<float>(2, 3));
    // printf("Rt2           = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", Rt2.at<float>(0, 0), Rt2.at<float>(0, 1), Rt2.at<float>(0, 2), Rt2.at<float>(0, 3), Rt2.at<float>(1, 0), Rt2.at<float>(1, 1), Rt2.at<float>(1, 2), Rt2.at<float>(1, 3), Rt2.at<float>(2, 0), Rt2.at<float>(2, 1), Rt2.at<float>(2, 2), Rt2.at<float>(2, 3));
    // printf("prjMat1       = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", prjMat1.at<float>(0, 0), prjMat1.at<float>(0, 1), prjMat1.at<float>(0, 2), prjMat1.at<float>(0, 3), prjMat1.at<float>(1, 0), prjMat1.at<float>(1, 1), prjMat1.at<float>(1, 2), prjMat1.at<float>(1, 3), prjMat1.at<float>(2, 0), prjMat1.at<float>(2, 1), prjMat1.at<float>(2, 2), prjMat1.at<float>(2, 3));
    // printf("prjMat2       = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", prjMat2.at<float>(0, 0), prjMat2.at<float>(0, 1), prjMat2.at<float>(0, 2), prjMat2.at<float>(0, 3), prjMat2.at<float>(1, 0), prjMat2.at<float>(1, 1), prjMat2.at<float>(1, 2), prjMat2.at<float>(1, 3), prjMat2.at<float>(2, 0), prjMat2.at<float>(2, 1), prjMat2.at<float>(2, 2), prjMat2.at<float>(2, 3));
    // printf("t_keyframe1   = [%0.4f %0.4f %0.4f]\n", t_keyframe1.at<float>(0, 0), t_keyframe1.at<float>(1, 0), t_keyframe1.at<float>(2, 0));
    // printf("t_arm         = [%0.4f %0.4f %0.4f]\n", t_arm.at<float>(0, 0), t_arm.at<float>(1, 0), t_arm.at<float>(2, 0));
    // printf("t_endo         = [%0.4f %0.4f %0.4f]\n", t_endo.at<float>(0, 0), t_endo.at<float>(1, 0), t_endo.at<float>(2, 0));
    // printf("t_32             = [%0.4f %0.4f %0.4f]\n", t_32.at<float>(0, 0), t_32.at<float>(1, 0), t_32.at<float>(2, 0));
    // printf("R_endo        = \n%0.4f %0.4f %0.4f\n%0.4f %0.4f %0.4f\n%0.4f %0.4f %0.4f\n", R_endo.at<float>(0, 0), R_endo.at<float>(0, 1), R_endo.at<float>(0, 2), R_endo.at<float>(1, 0), R_endo.at<float>(1, 1), R_endo.at<float>(1, 2), R_endo.at<float>(2, 0), R_endo.at<float>(2, 1), R_endo.at<float>(2, 2));
    cv::Mat point3D_arm_result = point3D_arm.reshape(1, 3);
    for (size_t i = 0; i < match_num; ++i)
    {
      // printf("point4D       #%zd = [%0.4f %0.4f %0.4f %0.4f]\n", i, point4D.at<float>(0, i), point4D.at<float>(1, i), point4D.at<float>(2, i), point4D.at<float>(3, i));
      printf("point3D       #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D.at<cv::Vec3f>(i, 0)[0], point3D.at<cv::Vec3f>(i, 0)[1], point3D.at<cv::Vec3f>(i, 0)[2]);
      // printf("point3D_arm   #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D_arm_result.at<float>(0, i), point3D_arm.at<float>(1, i), point3D_arm.at<float>(2, i));
    }
    printf("dist_mean = %0.4f", dist_mean);
    ////  reconstruction 終了 ////

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    convert_frame_to_message(cvframe_, atoi(msg_image->header.frame_id.c_str()), *msg_pub); //cv → msg
    // convert_pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);
    convert_pointcloud_to_PCL(pointCloud_arm, *msg_cloud_pub, 0);

    pub->publish(std::move(msg_pub));
    pub_pointcloud->publish(std::move(msg_cloud_pub));
    //display_tf(t_sum, r_sum, node); //tfで5点アルゴリズムで推定したカメラff座標をtfに変換およびパブリッシュ
  }

  //Keyframeの更新
  int frame_span = frame_num - frame_key2;
  if (frame_span > 10 && (cv::norm(t_arm2) > 8. || phi2 > M_PI / 480.))
  {
    dst_keyframe1 = dst_keyframe2.clone();
    keypoints_keyframe1 = keypoints_keyframe2;
    descriptor_keyframe1 = descriptor_keyframe2.clone();
    R_keyframe1 = R_keyframe2.clone();
    t_keyframe1 = t_keyframe2.clone();
    frame_key1 = frame_key2;

    dst_keyframe2 = dst_frame.clone();
    keypoints_keyframe2 = keypoints_frame;
    descriptor_keyframe2 = descriptor_frame.clone();
    R_keyframe2 = R_frame.clone();
    t_keyframe2 = t_frame.clone();
    frame_key2 = frame_num;
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

  bool show_camera = false;
  size_t feature = 0;
  size_t match = 0;
  size_t prjMat = 1;

  std::string topic_sub("endoscope_image");
  std::string topic_sub_arm("arm_trans");
  std::string topic_pub("feature_point");
  std::string topic_pub_pointcloud("pointcloud");

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
    cv::namedWindow("cvframe_", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("rec1", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("rec2", cv::WINDOW_AUTOSIZE);
  }
  // Initialize a ROS node.
  auto node = rclcpp::Node::make_shared("reconstruction");
  rclcpp::Logger node_logger = node->get_logger();

  //時間管理
  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  tf2_ros::Buffer buffer_(clock);
  tf2::TimePoint timepoint;
  tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
  buffer_.canTransform("endoscope", "world", tf2::TimePoint(), tf2::durationFromSec(1.0));

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  //Set QoS to Publish
  RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
  RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);                             // Create the image publisher with our custom QoS profile.
  auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
  message_filters::Subscriber<geometry_msgs::msg::Transform> arm_sub(node.get(), topic_sub_arm);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync(image_sub, arm_sub, 100);
  sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, prjMat, node_logger, pub, pub_pointcloud, &buffer_));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}