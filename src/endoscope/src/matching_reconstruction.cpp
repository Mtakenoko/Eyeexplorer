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
void transformRotation3D(cv::Mat InputMatrix, float rotation_x, float rotation_y, float rotation_z, cv::Mat &OutputMatrix)
{
  cv::Mat rot_x = (cv::Mat_<float>(3, 3) << 1., 0., 0., 0., std::cos(rotation_x), -std::sin(rotation_x), 0.0, std::sin(rotation_x), std::cos(rotation_x));
  cv::Mat rot_y = (cv::Mat_<float>(3, 3) << std::cos(rotation_y), 0.0, std::sin(rotation_y), 0.0, 1., 0., -std::sin(rotation_y), 0.0, std::cos(rotation_y));
  cv::Mat rot_z = (cv::Mat_<float>(3, 3) << std::cos(rotation_z), -std::sin(rotation_z), 0., std::sin(rotation_z), std::cos(rotation_z), 0., 0., 0., 1.);
  OutputMatrix = rot_x * rot_y * rot_z * InputMatrix;
  printf("unko\n");
}
void transformRobotToEndoscope(cv::Mat &OutputMatrix)
{
  cv::Mat InputMatrix = OutputMatrix.clone();
  transformRotation3D(InputMatrix, -PI / 2., 0., -PI / 2., OutputMatrix);
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
void convertCVMattoKtlMatrix(cv::Mat r, Ktl::Matrix<3, 3> &R)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R[i][j] = r.at<float>(i, j);
    }
  }
}
void convertCVMattoKtlMatrix2(
    float &R11, float &R12, float &R13,
    float &R21, float &R22, float &R23,
    float &R31, float &R32, float &R33,
    float r11, float r12, float r13,
    float r21, float r22, float r23,
    float r31, float r32, float r33)
{
  R11 = r11;
  R12 = r12;
  R13 = r13;
  R21 = r21;
  R22 = r22;
  R23 = r23;
  R31 = r31;
  R32 = r32;
  R33 = r33;
}

void display_tf(cv::Mat t_sum, cv::Mat r_sum, std::shared_ptr<rclcpp::Node> node)
{
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped msg;
  float qx, qy, qz, qw;
  transformRotMatToQuaternion(qx, qy, qz, qw,
                              r_sum.at<float>(0, 0), r_sum.at<float>(1, 0), r_sum.at<float>(2, 0),
                              r_sum.at<float>(0, 1), r_sum.at<float>(1, 1), r_sum.at<float>(2, 1),
                              r_sum.at<float>(0, 2), r_sum.at<float>(1, 2), r_sum.at<float>(2, 2));
  msg.transform.translation.x = t_sum.at<float>(0);
  msg.transform.translation.y = t_sum.at<float>(1);
  msg.transform.translation.z = t_sum.at<float>(2);
  msg.transform.rotation.x = qx;
  msg.transform.rotation.y = qy;
  msg.transform.rotation.z = qz;
  msg.transform.rotation.w = qw;
  msg.header.frame_id = "world";
  msg.child_frame_id = "camera_5point";
  broadcaster.sendTransform(msg);
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image, const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm, bool show_camera, size_t feature, size_t match, size_t prjMat, rclcpp::Logger logger, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  //ループカウンタ
  static int loop_count = 0;
  loop_count++;

  //グローバル座標とカメラ座標間の座標変換行列
  cv::Mat arm_trans(3, 1, CV_32FC1);
  cv::Mat arm_rot(3, 3, CV_32FC1);
  //並進成分
  arm_trans.at<float>(0) = msg_arm->translation.x;
  arm_trans.at<float>(1) = msg_arm->translation.y;
  arm_trans.at<float>(2) = msg_arm->translation.z;
  //回転成分
  transformQuaternionToRotMat(arm_rot.at<float>(0, 0), arm_rot.at<float>(0, 1), arm_rot.at<float>(0, 2),
                              arm_rot.at<float>(1, 0), arm_rot.at<float>(1, 1), arm_rot.at<float>(1, 2),
                              arm_rot.at<float>(2, 0), arm_rot.at<float>(2, 1), arm_rot.at<float>(2, 2),
                              msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);

  //// Key Frame 挿入 START ////
  //アーム姿勢をcv::Mat形式に変換
  static cv::Mat x1(3, 1, CV_32FC1), x2(3, 1, CV_32FC1);
  static cv::Mat r1(3, 3, CV_32FC1), r2(3, 3, CV_32FC1);
  x2 = arm_trans.clone();
  r2 = arm_rot.clone();
  if (loop_count == 1)
  {
    x1 = x2.clone();
    r1 = r2.clone();
    return;
  }
  //画像間でのカメラの移動量（運動学より）
  cv::Mat R_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
  R_arm = r2 * r1.t();
  t_arm = x2 - x1;

  //回転行列から回転角を取り出す
  //回転行列をクォータニオンに変換
  float qx, qy, qz, qw;
  transformRotMatToQuaternion(qx, qy, qz, qw,
                              R_arm.at<float>(0, 0), R_arm.at<float>(0, 1), R_arm.at<float>(0, 2),
                              R_arm.at<float>(1, 0), R_arm.at<float>(1, 1), R_arm.at<float>(1, 2),
                              R_arm.at<float>(2, 0), R_arm.at<float>(2, 1), R_arm.at<float>(2, 2));
  //クォータニオンの4つめの要素から回転角を取り出す
  float phi = 2 * std::acos(qw);

  //frame間隔
  static int frame1;
  if (loop_count == 1)
  {
    frame1 = atoi(msg_image->header.frame_id.c_str());
  }
  int frame2 = atoi(msg_image->header.frame_id.c_str());
  int frame_span = frame2 - frame1;

  //KeyFrame挿入タイミング
  if (!(frame_span > 10 && (cv::norm(t_arm) > 8. || phi > PI / 180.)))
  {
    printf("frame_span = %d, t_norm = %lf, phi = %f\n", frame_span, cv::norm(t_arm), phi * 180. / PI);
    return;
  }
  //// Key Frame 挿入 END ////

  //Subscribe
  cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
  if (msg_image->encoding == "rgb8")
  {
    cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
  }

  ////  detection 開始  ////
  static cv::Mat dst1, descriptor1;
  static std::vector<cv::KeyPoint> keypoints1;
  cv::Mat dst2, descriptor2;
  std::vector<cv::KeyPoint> keypoints2;
  cv::Ptr<cv::FeatureDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

  dst2 = frame_image.clone();
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
  detector->detect(dst2, keypoints2);
  descriptorExtractor->compute(dst2, keypoints2, descriptor2);

  //特徴抽出初めてやったときはdst1になんにも入ってない
  static bool set_bool = false;
  if (!set_bool)
  {
    dst1 = dst2.clone();
    keypoints1 = keypoints2;
    descriptor1 = descriptor2.clone();
    x1 = x2.clone();
    r1 = r2.clone();
    frame1 = frame2;
    set_bool = true;
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

  std::vector<cv::DMatch> dmatch, good_dmatch;
  std::vector<cv::DMatch> dmatch12, dmatch21;
  std::vector<cv::Point2f> match_point1, match_point2;

  //マッチング
  matcher->match(descriptor1, descriptor2, dmatch12); //dst1 -> dst2
  matcher->match(descriptor2, descriptor1, dmatch21); //dst2 -> dst1

  //dst1 -> dst2 と dst2 -> dst1の結果が一致しているか検証
  size_t good_count = 0;
  for (size_t i = 0; i < dmatch12.size(); ++i)
  {
    cv::DMatch m12 = dmatch12[i];
    cv::DMatch m21 = dmatch21[m12.trainIdx];

    if (m21.trainIdx == m12.queryIdx)
    { // 一致しているものだけを抜粋
      dmatch.push_back(m12);
      match_point1.push_back(keypoints1[dmatch12[i].queryIdx].pt);
      match_point2.push_back(keypoints2[dmatch12[i].trainIdx].pt);
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
    cameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0, 0.0, fovy, v0, 0.0, 0.0, 1.0);
    cameraMatrix.convertTo(cameraMatrix_64, CV_64FC1);

    //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
    std::vector<cv::Point2f> p1, p2;
    for (size_t i = 0; i < match_num; i++)
    {                             //特徴点の数だけ処理
      cv::Mat ip(3, 1, CV_64FC1); //3×1のベクトル
      cv::Point2f p_1, p_2;

      //ip.at<double>(0) = keypoints1[dmatch[i].queryIdx].pt.x;	//1枚目の画像の特徴点のx座標を取得
      //ip.at<double>(1) = keypoints1[dmatch[i].queryIdx].pt.y;	//1枚目の画像の特徴点のy座標を取得
      //ip.at<double>(2) = 1.0;	//同次座標用

      //ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      //p.x = ip.at<double>(0);
      //p.y = ip.at<double>(1);
      p_1.x = keypoints1[good_dmatch[i].queryIdx].pt.x;
      p_1.y = keypoints1[good_dmatch[i].queryIdx].pt.y;
      p1.push_back(p_1);

      //ip.at<double>(0) = keypoints2[dmatch[i].trainIdx].pt.x;	//2枚目の画像の特徴点のx座標を取得
      //ip.at<double>(1) = keypoints2[dmatch[i].trainIdx].pt.y;	//2枚目の画像の特徴点のy座標を取得
      //ip.at<double>(2) = 1.0;

      //ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
      //p.x = ip.at<double>(0);
      //p.y = ip.at<double>(1);
      p_2.x = keypoints2[good_dmatch[i].queryIdx].pt.x;
      p_2.y = keypoints2[good_dmatch[i].queryIdx].pt.y;
      p2.push_back(p_2);
    }
    //printf("t_arm = [%0.2f %0.2f %0.2f]\n", t_arm.at<float>(0), t_arm.at<float>(1), t_arm.at<float>(2));

    //5点アルゴリズム
    //１つめの画像を正規化座標としたときに2枚目の画像への回転・並進変換行列
    cv::Mat r(3, 3, CV_64FC1), t(3, 1, CV_64FC1);
    cv::Mat maskmask;                                                                                                 //RANSACの結果を保持するためのマスク
    cv::Mat essentialMat = cv::findEssentialMat(p1, p2, 1.0, cv::Point2f(0, 0), cv::RANSAC, 0.9999, 0.003, maskmask); //RANSACによって
    cv::recoverPose(essentialMat, p1, p2, r, t);
    //5点アルゴリズムで求めた座標をtfで表示するために移動量を累積する
    cv::Mat r_32(3, 3, CV_32FC1), t_32(3, 1, CV_32FC1);
    static cv::Mat t_sum(3, 1, CV_32FC1), r_sum(3, 3, CV_32FC1);
    r.convertTo(r_32, CV_32FC1);
    t.convertTo(t_32, CV_32FC1);
    static bool sum_state = false;
    if (!sum_state)
    {
      r_sum = r2.clone();
      t_sum = x2.clone();
      sum_state = true;
    }
    else
    {
      t_sum += t_32;
      r_sum = r_32 * r_sum;
    }

    //三角測量のためのカメラの透視射影行列
    cv::Mat prjMat1(3, 4, CV_32FC1), prjMat2(3, 4, CV_32FC1), out_prjMat1(3, 4, CV_64FC1), out_prjMat2(3, 4, CV_64FC1);
    cv::Mat Rt1(3, 4, CV_32FC1), Rt2(3, 4, CV_32FC1);
    cv::Mat R_arm_64(3, 3, CV_64FC1), t_arm_64(3, 1, CV_64FC1);
    cv::Mat out_R1, out_R2, out_Q;
    cv::Mat out_prjMat1_32, out_prjMat2_32;
    cv::Mat k(5, 1, CV_64FC1);

    if (prjMat == 0)
    {                                     //５点アルゴリズムで推定したカメラの位置姿勢から三角測量をする（カメラ空間）
      Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          Rt2.at<float>(i, j) = r_32.at<float>(i, j);
        }
        Rt2.at<float>(i, 3) = t_32.at<float>(i, 0);
      }
      prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
      prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列
      //k = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
      k = (cv::Mat_<double>(5, 1) << -0.303, -0.200272, -0.004333, -0.002127, 0.0);
      cv::stereoRectify(cameraMatrix_64, k, cameraMatrix_64, k, cv::Size(320, 320), r, t, out_R1, out_R2, out_prjMat1, out_prjMat2, out_Q);
      out_prjMat1.convertTo(out_prjMat1_32, CV_32FC1);
      out_prjMat2.convertTo(out_prjMat2_32, CV_32FC1);
    }
    else if (prjMat == 1)
    {                                     //運動学から求めたカメラの位置姿勢から三角測量をする（カメラ空間）
      Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0の外部パラメータ
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          Rt2.at<float>(i, j) = R_arm.at<float>(i, j);
        }
        Rt2.at<float>(i, 3) = t_arm.at<float>(i, 0);
      }
      prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
      prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列

      R_arm.convertTo(R_arm_64, CV_64FC1);
      t_arm.convertTo(t_arm_64, CV_64FC1);
      //k = (cv::Mat_<double>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
      k = (cv::Mat_<double>(5, 1) << -0.303, -0.200272, -0.004333, -0.002127, 0.0);
      cv::stereoRectify(cameraMatrix_64, k, cameraMatrix_64, k, cv::Size(320, 320), R_arm_64, t_arm_64, out_R1, out_R2, out_prjMat1, out_prjMat2, out_Q);
      out_prjMat1.convertTo(out_prjMat1_32, CV_32FC1);
      out_prjMat2.convertTo(out_prjMat2_32, CV_32FC1);
    }
    else if (prjMat == 2)
    { //運動学から求めたカメラの位置姿勢から三角測量をする（ワールド座標）
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
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
    cv::Mat point4D(4, match_num, CV_32FC1), point4D_(4, match_num, CV_32FC1), point3D, point3D_;
    cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point4D);
    cv::triangulatePoints(out_prjMat1_32, out_prjMat2_32, p1, p2, point4D_);
    cv::convertPointsFromHomogeneous(point4D.t(), point3D);
    cv::convertPointsFromHomogeneous(point4D_.t(), point3D_);
    //prjMat1 : 運動学で求めたカメラ空間上での動作前のカメラの位置・姿勢行列
    //prjMat2 : 運動学で求めたカメラ空間上での動作後のカメラの位置・姿勢行列
    //p1      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間にで表現したもの
    //p2      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間で表現したもの
    //point4D : 三角測量で求めた各特徴点座標をカメラ空間にて表現したもの

    // RViz2用に並べ替え
    cv::Mat pointCloud(match_num, 1, CV_32FC3);
    cv::Mat pointCloud_arm(match_num, 1, CV_32FC3);
    point3D.convertTo(pointCloud, CV_32FC3);
    /*
    //カメラ座標系　→　ワールド座標系
    cv::Mat point3D_shoih;
    std::vector<cv::Point3f> point3D_wa;
    for (size_t i = 0; i < match_num; ++i)
    {    
      cv::Point3f point_world;
      point_world.x = pointCloud.at<cv::Vec3f>(i, 0)[0] + t_arm.at<float>(0, 0);
      point_world.y = pointCloud.at<cv::Vec3f>(i, 0)[1] + t_arm.at<float>(1, 0);
      point_world.z = pointCloud.at<cv::Vec3f>(i, 0)[2] + t_arm.at<float>(2, 0);

      cv::Mat point3D_w_ = R_arm.t() * pointCloud.at<float>(i);// + t_arm; //カメラ座標系　→　ワールド座標系に変換
      point3D_shoih.push_back(point3D_w_);
      point3D_wa.push_back(point_world);
    }
    cv::Mat point3D_w = point3D_shoih.reshape(3, match_num);*/
    /*
    //カメラ座標系　→　ワールド座標系
    cv::Mat point3D_cam = pointCloud.reshape(1, 3);
    cv::Mat t_w(3, match_num, CV_32FC1);
    for (size_t i = 0; i < match_num; ++i)
    {
      t_w.at<cv::Vec3f>(i) = t_arm.at<cv::Vec3f>(0);
    }
    cv::Mat point3D_w = R_arm * point3D_cam + t_w; //カメラ座標系　→　ワールド座標系に変換
    cv::Mat pointCloud_W = point3D_w.reshape(3, match_num);
    */
    //距離（ノルム）が大きすぎる&負の値は誤対応と判断し除去
    int dist_count = 0;
    for (size_t i = 0; i < match_num; ++i)
    {
      if (pointCloud.at<cv::Vec3f>(i, 0)[2] > 0. && pointCloud.at<cv::Vec3f>(i, 0)[2] < 1000.)
      {
        for (size_t j = 0; j < 3; ++j)
        {
          pointCloud_arm.at<cv::Vec3f>(i - dist_count, 0)[j] = pointCloud.at<cv::Vec3f>(i, 0)[j];
        }
      }
      else
      {
        dist_count++;
      }
    }

    //cv::imshow
    cv::Mat cvframe, cvframe_, cvframe_t;
    cv::drawMatches(dst1, keypoints1, dst2, keypoints2, dmatch, cvframe);
    cv::drawMatches(dst1, keypoints1, dst2, keypoints2, good_dmatch, cvframe_);
    if (show_camera)
    {
      cv::Point2f center_t, center_t_arm;
      cv::Point2f p1 = cv::Point2f(msg_image->height / 2., msg_image->width / 2.);
      cv::Scalar color = cv::Scalar(0, 255, 0);
      cv::Mat t_endo(3, 1, CV_32FC1);
      t_endo = r1 * t_arm;
      center_t = cv::Point2f(t.at<float>(0, 0) * 20 + p1.x, t.at<float>(1, 0) * 20 + p1.y);
      center_t_arm = cv::Point2f(t_endo.at<float>(0) * 20 + p1.x, t_endo.at<float>(1) * 20 + p1.y);
      if (prjMat == 0)
      {
        cvframe_t = cvframe_.clone();
        cv::arrowedLine(cvframe, p1, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_, p1, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_t, p1, center_t, color, 2, 8, 0, 0.5);
        cv::imshow("cvframe", cvframe);
        cv::imshow("cvframe_", cvframe_);
        cv::imshow("cvframe_t", cvframe_t);
      }
      else
      {
        cv::arrowedLine(cvframe, p1, center_t_arm, color, 2, 8, 0, 0.5);
        cv::arrowedLine(cvframe_, p1, center_t_arm, color, 2, 8, 0, 0.5);
        cv::imshow("cvframe", cvframe);
        cv::imshow("cvframe_", cvframe_);
      }
      cv::waitKey(1);
    }

    //変数のサイズ確認
    //printf("x1 = [%0.4f %0.4f %0.4f]\n", x1.at<float>(0), x1.at<float>(1), x1.at<float>(2));
    //printf("dmatch: %zu\n", match_num);
    //printf("point4D       : %d %d %d\n", point4D.rows, point4D.cols, point4D.channels());
    //printf("point3D       : %d %d %d\n", point3D.rows, point3D.cols, point3D.channels());
    //printf("point3D_      : %d %d %d\n", point3D_.rows, point3D_.cols, point3D_.channels());
    //printf("pointCloud    : %d %d %d\n", pointCloud.rows, pointCloud.cols, pointCloud.channels());
    //printf("point3D_cam   : %d %d %d\n", point3D_cam.rows, point3D_cam.cols, point3D_cam.channels());
    //printf("t_w           : %d %d %d\n", t_w.rows, t_w.cols, t_w.channels());
    //printf("point3D_w     : %d %d %d\n", point3D_w.rows, point3D_w.cols, point3D_w.channels());
    //printf("pointCloud_w  : %d %d %d\n", pointCloud_W.rows, pointCloud_W.cols, pointCloud_W.channels());
    //printf("point3D_shoih  : %d %d %d\n", point3D_shoih.rows, point3D_shoih.cols, point3D_shoih.channels());

    //printf
    //printf("Rt1           = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", Rt1.at<float>(0,0), Rt1.at<float>(0,1), Rt1.at<float>(0,2), Rt1.at<float>(0,3), Rt1.at<float>(1,0), Rt1.at<float>(1,1), Rt1.at<float>(1,2), Rt1.at<float>(1,3), Rt1.at<float>(2,0), Rt1.at<float>(2,1), Rt1.at<float>(2,2), Rt1.at<float>(2,3));
    //printf("Rt2           = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", Rt2.at<float>(0,0), Rt2.at<float>(0,1), Rt2.at<float>(0,2), Rt2.at<float>(0,3), Rt2.at<float>(1,0), Rt2.at<float>(1,1), Rt2.at<float>(1,2), Rt2.at<float>(1,3), Rt2.at<float>(2,0), Rt2.at<float>(2,1), Rt2.at<float>(2,2), Rt2.at<float>(2,3));
    //printf("prjMat1       = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", prjMat1.at<float>(0,0), prjMat1.at<float>(0,1), prjMat1.at<float>(0,2), prjMat1.at<float>(0,3), prjMat1.at<float>(1,0), prjMat1.at<float>(1,1), prjMat1.at<float>(1,2), prjMat1.at<float>(1,3), prjMat1.at<float>(2,0), prjMat1.at<float>(2,1), prjMat1.at<float>(2,2), prjMat1.at<float>(2,3));
    //printf("out_prjMat1   = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", out_prjMat1_32.at<float>(0,0), out_prjMat1_32.at<float>(0,1), out_prjMat1_32.at<float>(0,2), out_prjMat1_32.at<float>(0,3), out_prjMat1_32.at<float>(1,0), out_prjMat1_32.at<float>(1,1), out_prjMat1_32.at<float>(1,2), out_prjMat1_32.at<float>(1,3), out_prjMat1_32.at<float>(2,0), out_prjMat1_32.at<float>(2,1), out_prjMat1_32.at<float>(2,2), out_prjMat1_32.at<float>(2,3));
    //printf("prjMat2       = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", prjMat2.at<float>(0,0), prjMat2.at<float>(0,1), prjMat2.at<float>(0,2), prjMat2.at<float>(0,3), prjMat2.at<float>(1,0), prjMat2.at<float>(1,1), prjMat2.at<float>(1,2), prjMat2.at<float>(1,3), prjMat2.at<float>(2,0), prjMat2.at<float>(2,1), prjMat2.at<float>(2,2), prjMat2.at<float>(2,3));
    //printf("out_prjMat2   = \n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n%0.5f %0.5f %0.5f %0.5f\n", out_prjMat2_32.at<float>(0,0), out_prjMat2_32.at<float>(0,1), out_prjMat2_32.at<float>(0,2), out_prjMat2_32.at<float>(0,3), out_prjMat2_32.at<float>(1,0), out_prjMat2_32.at<float>(1,1), out_prjMat2_32.at<float>(1,2), out_prjMat2_32.at<float>(1,3), out_prjMat2_32.at<float>(2,0), out_prjMat2_32.at<float>(2,1), out_prjMat2_32.at<float>(2,2), out_prjMat2_32.at<float>(2,3));
    //printf("t_arm         = [%0.4f %0.4f %0.4f]\n", t_arm.at<float>(0, 0), t_arm.at<float>(1, 0), t_arm.at<float>(2, 0));
    //printf("t             = [%0.4f %0.4f %0.4f]\n", t.at<float>(0, 0), t.at<float>(1, 0), t.at<float>(2, 0));
    for (size_t i = 0; i < match_num; ++i)
    {
      printf("point4D       #%zd = [%0.4f %0.4f %0.4f %0.4f]\n", i, point4D.at<float>(0, i), point4D.at<float>(1, i), point4D.at<float>(2, i), point4D.at<float>(3, i));
      printf("point4D_      #%zd = [%0.4f %0.4f %0.4f %0.4f]\n", i, point4D_.at<float>(0, i), point4D_.at<float>(1, i), point4D_.at<float>(2, i), point4D_.at<float>(3, i));
      printf("point3D       #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D.at<cv::Vec3f>(i, 0)[0], point3D.at<cv::Vec3f>(i, 0)[1], point3D.at<cv::Vec3f>(i, 0)[2]);
      printf("point3D_      #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D_.at<cv::Vec3f>(i, 0)[0], point3D_.at<cv::Vec3f>(i, 0)[1], point3D_.at<cv::Vec3f>(i, 0)[2]);
      printf("pointCloud    #%zd = [%0.4f %0.4f %0.4f]\n", i, pointCloud.at<cv::Vec3f>(i, 0)[0], pointCloud.at<cv::Vec3f>(i, 0)[1], pointCloud.at<cv::Vec3f>(i, 0)[2]);
      //printf("point3D_w    #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D_w.at<cv::Vec3f>(i, 0)[0], point3D_w.at<cv::Vec3f>(i, 0)[1], point3D_w.at<cv::Vec3f>(i, 0)[2]);
      //if (i < pointCloud_arm.rows)
      //printf("pointCloud_arm #%zd = [%0.4f %0.4f %0.4f]\n", i, pointCloud_arm.at<cv::Vec3f>(i, 0)[0], pointCloud_arm.at<cv::Vec3f>(i, 0)[1], pointCloud_arm.at<cv::Vec3f>(i, 0)[2]);

      //printf("t_w         #%zd = [%0.4f %0.4f %0.4f]\n", i, t_w.at<float>(0, i), t_w.at<float>(1, i), t_w.at<float>(2, i));
    }
    ////  reconstruction 終了 ////

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    convert_frame_to_message(cvframe, atoi(msg_image->header.frame_id.c_str()), *msg_pub); //cv → msg
    //convert_pointcloud_to_PCL(pointCloud2, *msg_cloud_pub, dist_count);
    //convert_pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);
    convert_pointcloud_to_PCL(pointCloud_arm, *msg_cloud_pub, 0);

    pub->publish(std::move(msg_pub));
    pub_pointcloud->publish(std::move(msg_cloud_pub));
    //display_tf(t_sum, r_sum, node); //tfで5点アルゴリズムで推定したカメラff座標をtfに変換およびパブリッシュ
  }

  // 1つめのフレームに代入
  dst1 = dst2.clone();
  keypoints1 = keypoints2;
  descriptor1 = descriptor2.clone();
  x1 = x2.clone();
  r1 = r2.clone();
  frame1 = frame2;
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

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);                             // Create the image publisher with our custom QoS profile.
  auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
  message_filters::Subscriber<geometry_msgs::msg::Transform> arm_sub(node.get(), topic_sub_arm);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync(image_sub, arm_sub, 100);
  sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, prjMat, node_logger, pub, pub_pointcloud));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}