#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../include/options_reconstruction_BA.hpp"
#include "../../HTL/include/transform.h"
#include "../../HTL/include/msg_converter.h"
#include "../include/endoscope/bundleadjustment.hpp"

Transform transform;
Converter converter;

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

void triangulation(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image, const std::shared_ptr<const geometry_msgs::msg::Transform> &msg_arm,
                   bool show_camera, size_t feature, size_t match, size_t prjMat, rclcpp::Logger logger,
                   std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
  RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());
  //ループカウンタ
  static int loop_count = 0;
  loop_count++;

  //処理時間計測
  std::chrono::system_clock::time_point start;
  std::chrono::system_clock::time_point end[10];
  start = std::chrono::system_clock::now();

  //グローバル座標とカメラ座標間の座標変換行列
  cv::Mat arm_trans(3, 1, CV_32FC1);
  cv::Mat arm_rot(3, 3, CV_32FC1);
  // アームの並進成分（[m]→[mm]にも変換）
  arm_trans.at<float>(0) = (float)msg_arm->translation.x * 1000.;
  arm_trans.at<float>(1) = (float)msg_arm->translation.y * 1000.;
  arm_trans.at<float>(2) = (float)msg_arm->translation.z * 1000.;
  // アームの回転成分
  transform.QuaternionToRotMat(arm_rot.at<float>(0, 0), arm_rot.at<float>(0, 1), arm_rot.at<float>(0, 2),
                               arm_rot.at<float>(1, 0), arm_rot.at<float>(1, 1), arm_rot.at<float>(1, 2),
                               arm_rot.at<float>(2, 0), arm_rot.at<float>(2, 1), arm_rot.at<float>(2, 2),
                               (float)msg_arm->rotation.x, (float)msg_arm->rotation.y, (float)msg_arm->rotation.z, (float)msg_arm->rotation.w);

  cv::Mat arm_rot_Quaternion = (cv::Mat_<double>(4, 1) << msg_arm->rotation.x, msg_arm->rotation.y, msg_arm->rotation.z, msg_arm->rotation.w);

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
  static cv::Mat R_keyframe_Quaternion(4, 1, CV_64FC1), R_keyframe1_Quaternion(4, 1, CV_64FC1), R_keyframe2_Quaternion(4, 1, CV_64FC1), R_frame_Quaternion(4, 1, CV_64FC1);
  static cv::Mat t_keyframe(3, 1, CV_32FC1), t_keyframe1(3, 1, CV_32FC1), t_keyframe2(3, 1, CV_32FC1), t_frame(3, 1, CV_32FC1);
  R_frame = arm_rot.clone();
  R_frame_Quaternion = arm_rot_Quaternion.clone();
  t_frame = arm_trans.clone();
  end[0] = std::chrono::system_clock::now();

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
  end[1] = std::chrono::system_clock::now();
  detector->detect(dst_frame, keypoints_frame);
  descriptorExtractor->compute(dst_frame, keypoints_frame, descriptor_frame);

  end[2] = std::chrono::system_clock::now();
  //特徴抽出初めてやったときはdst_keyframe1になんにも入ってない
  static bool set_keyframe1 = false, set_keyframe2 = false;
  if (!set_keyframe1)
  {
    //keyframe1の設定
    dst_keyframe1 = dst_frame.clone();
    keypoints_keyframe1 = keypoints_frame;
    descriptor_keyframe1 = descriptor_frame.clone();
    R_keyframe1 = R_frame.clone();
    R_keyframe1_Quaternion = R_frame_Quaternion.clone();
    t_keyframe1 = t_frame.clone();
    frame_key1 = frame_num;
    set_keyframe1 = true;
    return;
  }
  if (!set_keyframe2)
  {
    //keyframe2の設定
    dst_keyframe2 = dst_frame.clone();
    keypoints_keyframe2 = keypoints_frame;
    descriptor_keyframe2 = descriptor_frame.clone();
    R_keyframe2 = R_frame.clone();
    R_keyframe2_Quaternion = R_frame_Quaternion.clone();
    t_keyframe2 = t_frame.clone();
    frame_key2 = frame_num;
    set_keyframe2 = true;
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
  R_arm1 = R_frame * R_keyframe1.t(); //取得フレームとキーフレーム1でのカメラの回転変化
  R_arm2 = R_frame * R_keyframe2.t(); //取得フレームとキーフレーム2でのカメラの回転変化
  t_arm1 = t_frame - t_keyframe1;     //取得__QuaternionQuaternion_Quaternionフレームとキーフレーム1でのワールド座標系でのカメラ移動量
  t_arm2 = t_frame - t_keyframe2;     //取得フレームとキーフレーム2でのワールド座標系でのカメラ移動量
  float phi1 = transform.RevFromRotMat(R_arm1);
  float phi2 = transform.RevFromRotMat(R_arm2);
  float Keyframedetect = std::abs(phi1) - std::abs(phi2);
  if (Keyframedetect > 0)
  {
    // printf("use Keyframe 1\n");
    R_arm = R_arm1.clone();
    t_arm = t_arm1.clone();
    R_keyframe = R_keyframe1.clone();
    R_keyframe_Quaternion = R_keyframe1_Quaternion.clone();
    t_keyframe = t_keyframe1.clone();
    dst_keyframe = dst_keyframe1;
    matcher->match(descriptor_keyframe1, descriptor_frame, dmatch12); //dst_keyframe1 -> dst_frame
    matcher->match(descriptor_frame, descriptor_keyframe1, dmatch21); //dst_frame -> dst_keyframe1
    keypoints_keyframe = keypoints_keyframe1;
  }
  else
  {
    // printf("use Keyframe 2\n");
    R_arm = R_arm2.clone();
    t_arm = t_arm2.clone();
    R_keyframe = R_keyframe2.clone();
    R_keyframe_Quaternion = R_keyframe2_Quaternion.clone();
    t_keyframe = t_keyframe2.clone();
    dst_keyframe = dst_keyframe2;
    matcher->match(descriptor_keyframe2, descriptor_frame, dmatch12); //dst_keyframe1 -> dst_frame
    matcher->match(descriptor_frame, descriptor_keyframe2, dmatch21); //dst_frame -> dst_keyframe1
    keypoints_keyframe = keypoints_keyframe2;
  }
  end[3] = std::chrono::system_clock::now();

  //ワールド座標から内視鏡座標系で表示（keyframeと現フレームの回転行列と並進移動）
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
  end[4] = std::chrono::system_clock::now();
  ////    matching 終了   ////

  ////    reconstruction 開始     ////
  size_t match_num = good_dmatch.size();
  if (match_num > 20) //５点アルゴリズムが行えるのに十分対応点があれば
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

    end[5] = std::chrono::system_clock::now();

    //三角測量
    cv::Mat point3D, point3D_arm, point3D_Arm, point3D_BA;
    cv::Mat point4D(4, match_num, CV_32FC1);
    float dist_mean;
    for (size_t i = 0; i < match_num; ++i)
    {
      cv::Mat point3D_result, point3D_result_arm;
      cv::triangulatePoints(prjMat1, prjMat2, cv::Mat(p1[i]), cv::Mat(p2[i]), point4D);
      cv::convertPointsFromHomogeneous(point4D.reshape(4, 1), point3D_result);
      point3D_result_arm = R_keyframe * point3D_result.reshape(1, 3) + t_keyframe;
      point3D.push_back(point3D_result);
      point3D_arm.push_back(point3D_result_arm.reshape(3, 1));
      dist_mean += point3D_result.at<cv::Vec3f>(0, 0)[2];
    }
    point3D_Arm = point3D_arm / 1000.; //Viz用にmmからｍに変換
    dist_mean = dist_mean / match_num;

    //バンドル調整(Ceres Solver)
    //最適化問題解くためのオブジェクト作成
    ceres::Problem problem;
    //バンドル調整用パラメータ
    double mutable_camera_for_observations[2][6];
    double mutable_point_for_observations[300][3];
    cv::Mat rvec_keyframe, rvec_frame;
    cv::Rodrigues(R_keyframe.t(), rvec_keyframe);
    cv::Rodrigues(R_frame.t(), rvec_frame);
    //KeyFrameの方の情報
    mutable_camera_for_observations[0][0] = (double)rvec_keyframe.at<float>(0);
    mutable_camera_for_observations[0][1] = (double)rvec_keyframe.at<float>(1);
    mutable_camera_for_observations[0][2] = (double)rvec_keyframe.at<float>(2);
    mutable_camera_for_observations[0][3] = (double)t_keyframe.at<float>(0);
    mutable_camera_for_observations[0][4] = (double)t_keyframe.at<float>(1);
    mutable_camera_for_observations[0][5] = (double)t_keyframe.at<float>(2);
    //Frameの方の情報
    mutable_camera_for_observations[1][0] = (double)rvec_frame.at<float>(0);
    mutable_camera_for_observations[1][1] = (double)rvec_frame.at<float>(1);
    mutable_camera_for_observations[1][2] = (double)rvec_frame.at<float>(2);
    mutable_camera_for_observations[1][3] = (double)t_frame.at<float>(0);
    mutable_camera_for_observations[1][4] = (double)t_frame.at<float>(1);
    mutable_camera_for_observations[1][5] = (double)t_frame.at<float>(2);

    for (size_t i = 0; i < match_num; i++)
    {
      mutable_point_for_observations[i][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
      mutable_point_for_observations[i][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
      mutable_point_for_observations[i][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];
      mutable_point_for_observations[i + match_num][0] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[0];
      mutable_point_for_observations[i + match_num][1] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[1];
      mutable_point_for_observations[i + match_num][2] = (double)point3D_arm.at<cv::Vec3f>(i, 0)[2];

      //コスト関数
      ceres::CostFunction *cost_function_query = SnavelyReprojectionError::Create((double)keypoints_keyframe[good_dmatch[i].queryIdx].pt.x, (double)keypoints_keyframe[good_dmatch[i].queryIdx].pt.y);
      ceres::CostFunction *cost_function_train = SnavelyReprojectionError::Create((double)keypoints_frame[good_dmatch[i].trainIdx].pt.x, (double)keypoints_frame[good_dmatch[i].trainIdx].pt.y);
      problem.AddResidualBlock(cost_function_query, NULL, mutable_camera_for_observations[0], mutable_point_for_observations[i]);
      problem.AddResidualBlock(cost_function_train, NULL, mutable_camera_for_observations[1], mutable_point_for_observations[i + match_num]);
    }

    //Solverのオプション選択
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 8;

    //Solve
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //レポートなど出力
    // std::cout << summary.FullReport() << "\n";
    std::cout << "rvec_x:" << rvec_frame.at<float>(0) << "->" << mutable_camera_for_observations[1][0] << std::endl;
    std::cout << "rvec_y:" << rvec_frame.at<float>(1) << "->" << mutable_camera_for_observations[1][1] << std::endl;
    std::cout << "rvec_z:" << rvec_frame.at<float>(2) << "->" << mutable_camera_for_observations[1][2] << std::endl;
    std::cout << "x     :" << t_frame.at<float>(0) << "->" << mutable_camera_for_observations[1][3] << std::endl;
    std::cout << "y     :" << t_frame.at<float>(1) << "->" << mutable_camera_for_observations[1][4] << std::endl;
    std::cout << "z     :" << t_frame.at<float>(2) << "->" << mutable_camera_for_observations[1][5] << std::endl;

    std::cout << "point_x:" << point3D_arm.at<cv::Vec3f>(0, 0)[0] << "->" << mutable_point_for_observations[0][0] << std::endl;
    std::cout << "point_y:" << point3D_arm.at<cv::Vec3f>(0, 0)[1] << "->" << mutable_point_for_observations[0][1] << std::endl;
    std::cout << "point_z:" << point3D_arm.at<cv::Vec3f>(0, 0)[2] << "->" << mutable_point_for_observations[0][2] << std::endl;
    for (size_t i = 0; i < match_num; ++i)
    {
      cv::Point3f p_xyz;
      p_xyz.x = (float)mutable_point_for_observations[i][0] / 1000.;
      p_xyz.y = (float)mutable_point_for_observations[i][1] / 1000.;
      p_xyz.z = (float)mutable_point_for_observations[i][2] / 1000.;
      point3D_BA.push_back(p_xyz);
    }

    // RViz2用に並べ替え
    cv::Mat pointCloud(match_num, 1, CV_32FC3), pointCloud_arm(match_num, 1, CV_32FC3), pointCloud_BA(match_num, 1, CV_32FC3);
    point3D.convertTo(pointCloud, CV_32FC3);
    point3D_Arm.convertTo(pointCloud_arm, CV_32FC3);
    point3D_BA.convertTo(pointCloud_BA, CV_32FC3);
    end[6] = std::chrono::system_clock::now();
    //cv::imshow
    if (show_camera)
    {
      cv::Mat cvframe, cvframe_, cvframe_t;
      cv::drawMatches(dst_keyframe, keypoints_keyframe, dst_frame, keypoints_frame, dmatch, cvframe);
      cv::drawMatches(dst_keyframe, keypoints_keyframe, dst_frame, keypoints_frame, good_dmatch, cvframe_);
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
    end[7] = std::chrono::system_clock::now();

    //変数のサイズ確認
    // printf("dmatch: %zu\n", match_num);
    // printf("t_keyframe1   : %d %d %d\n", t_keyframe1.rows, t_keyframe1.cols, t_keyframe1.channels());
    // printf("Rt1           : %d %d %d\n", Rt1.rows, Rt1.cols, Rt1.channels());
    // printf("point4D       : %d %d %d\n", point4D.rows, point4D.cols, point4D.channels());
    // printf("point3D       : %d %d %d\n", point3D.rows, point3D.cols, point3D.channels());
    // printf("pointCloud    : %d %d %d\n", pointCloud.rows, pointCloud.cols, pointCloud.channels());
    // printf("imagePoints   : %d %d %d\n", imagePoints_keyframe.rows, imagePoints_keyframe.cols, imagePoints_keyframe.channels());

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
    // printf("dist_mean = %0.4f\n", dist_mean);
    // cv::Mat point3D_arm_result = point3D_arm.reshape(1, 3);
    for (size_t i = 0; i < match_num; ++i)
    {
      // printf("point4D       #%zd = [%0.4f %0.4f %0.4f %0.4f]\n", i, point4D.at<float>(0, i), point4D.at<float>(1, i), point4D.at<float>(2, i), point4D.at<float>(3, i));
      // printf("point3D       #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D.at<cv::Vec3f>(i, 0)[0], point3D.at<cv::Vec3f>(i, 0)[1], point3D.at<cv::Vec3f>(i, 0)[2]);
      // printf("point3D_arm   #%zd = [%0.4f %0.4f %0.4f]\n", i, point3D_arm_result.at<float>(0, i), point3D_arm.at<float>(1, i), point3D_arm.at<float>(2, i));
    }
    ////  reconstruction 終了 ////

    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // converter.pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);
    // converter.pointcloud_to_PCL(pointCloud_arm, *msg_cloud_pub, 0);
    converter.pointcloud_to_PCL(pointCloud_BA, *msg_cloud_pub, 0);

    pub_pointcloud->publish(std::move(msg_cloud_pub));
  }
  end[8] = std::chrono::system_clock::now();

  //Keyframeの更新
  int frame_span = frame_num - frame_key2;
  if (frame_span > 10 && (cv::norm(t_arm2) > 3. || phi2 > M_PI / 540.)) //@TODO   角度のtanと並進移動の和で判定を行うべき
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
    printf("Keyframe was changed!\n");
  }
  end[9] = std::chrono::system_clock::now();
  // //処理時間表示
  // double time[10], total;
  // time[0] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end[0] - start).count() / 1000.0);
  // printf("time = [%0.2lf ", time[0]);
  // for (int i = 1; i < 10; i++)
  // {
  //   time[i] = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end[i] - end[i - 1]).count() / 1000.0);
  //   printf("%0.2lf ", time[i]);
  // }
  // printf("]\n");
  // total = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end[9] - start).count() / 1000.0);
  // printf("total = %0.1lf, FPS = %0.2lf\n", total, 1000. / total);
}

int main(int argc, char *argv[])
{
  // Pass command line arguments to rclcpp.
  rclcpp::init(argc, argv);

  //
  google::InitGoogleLogging(argv[0]);

  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

  bool show_camera = false;
  size_t feature = 0;
  size_t match = 0;
  size_t prjMat = 1;

  std::string topic_sub("endoscope_image");
  std::string topic_sub_arm("endoscope_transform");
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
  auto node = rclcpp::Node::make_shared("reconstruction_BA");
  rclcpp::Logger node_logger = node->get_logger();

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  //Set QoS to Publish
  RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());

  auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub(node.get(), topic_sub);
  message_filters::Subscriber<geometry_msgs::msg::Transform> arm_sub(node.get(), topic_sub_arm);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, geometry_msgs::msg::Transform> sync(image_sub, arm_sub, 10000);
  sync.registerCallback(std::bind(&triangulation, std::placeholders::_1, std::placeholders::_2, show_camera, feature, match, prjMat, node_logger, pub_pointcloud));

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}