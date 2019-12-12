#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

#include <ktl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "../include/options_stereo_reconstruction.hpp"

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

void convert_pointcloud_to_PCL(const cv::Mat pointCloud, sensor_msgs::msg::PointCloud2 &msg_cloud_pub, int dist_count)
{
    msg_cloud_pub.header = std_msgs::msg::Header();
    msg_cloud_pub.header.stamp = rclcpp::Clock().now();
    msg_cloud_pub.header.frame_id = "world";

    msg_cloud_pub.is_bigendian = false;
    msg_cloud_pub.is_dense = true;

    msg_cloud_pub.height = 1;
    msg_cloud_pub.width = pointCloud.rows;

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
            floatData[i * (msg_cloud_pub.point_step / sizeof(float)) + j] = pointCloud.at<cv::Vec3f>(i)[j];
        }
    }
}

cv::Mat convert_handsystem_rot(const cv::Mat Input_Rot)
{
    cv::Mat Output_Rot;
    Output_Rot = Input_Rot.clone();
    Output_Rot.at<float>(3) = -1 * Input_Rot.at<float>(3);
    return Output_Rot;
}
cv::Mat convert_handsystem_t(const cv::Mat Input_t)
{
    cv::Mat Output_t;
    Output_t = Input_t.clone();
    Output_t.at<float>(3) = -1 * Input_t.at<float>(3);
    return Output_t;
}

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image_L, const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image_R, bool show_camera, size_t ransaq, size_t feature, size_t match, rclcpp::Logger logger, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud)
{
    RCLCPP_INFO(logger, "Received image #%s", msg_image_L->header.frame_id.c_str());

    //Subscribe
    cv::Mat frame_image_L(msg_image_L->height, msg_image_L->width, encoding2mat_type(msg_image_L->encoding), const_cast<unsigned char *>(msg_image_L->data.data()), msg_image_L->step);
    cv::Mat frame_image_R(msg_image_R->height, msg_image_R->width, encoding2mat_type(msg_image_R->encoding), const_cast<unsigned char *>(msg_image_R->data.data()), msg_image_R->step);

    if (msg_image_L->encoding == "rgb8")
    {
        cv::cvtColor(frame_image_L, frame_image_L, cv::COLOR_RGB2BGR);
    }
    if (msg_image_R->encoding == "rgb8")
    {
        cv::cvtColor(frame_image_R, frame_image_R, cv::COLOR_RGB2BGR);
    }

    ////  detection 開始  ////
    cv::Mat dst_L, dst_R;
    std::vector<cv::KeyPoint> keypoints_L, keypoints_R;
    cv::Mat descriptor_L, descriptor_R;

    dst_L = frame_image_L.clone();
    dst_R = frame_image_R.clone();
    cv::Ptr<cv::FeatureDetector> detector_L, detector_R;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_L, descriptorExtractor_R;
    if (feature == 0)
    {
        // AKAZE特徴抽出
        detector_L = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0008f);
        detector_R = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0008f);
        descriptorExtractor_L = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0008f);
        descriptorExtractor_R = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0008f);
    }
    else if (feature == 1)
    {
        //ORB特徴量抽出
        detector_L = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
        detector_R = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
        descriptorExtractor_L = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
        descriptorExtractor_R = cv::ORB::create(1000, 1.2f, 30, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 5);
    }
    else if (feature == 2)
    {
        //BRISK特徴量抽出
        detector_L = cv::BRISK::create(120, 3, 0.6f);
        detector_R = cv::BRISK::create(120, 3, 0.6f);
        descriptorExtractor_L = cv::BRISK::create(120, 3, 0.6f);
        descriptorExtractor_R = cv::BRISK::create(120, 3, 0.6f);
    }
    else
    {
        printf("Choosing Incorrect Option of Feature point detector.\n");
        return;
    }
    detector_L->detect(dst_L, keypoints_L);
    detector_R->detect(dst_R, keypoints_R);
    descriptorExtractor_L->compute(dst_L, keypoints_L, descriptor_L);
    descriptorExtractor_R->compute(dst_R, keypoints_R, descriptor_R);

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

    std::vector<cv::DMatch> dmatch, dmatch12, dmatch21, Dmatch;
    std::vector<cv::Point2f> match_point1, match_point2;

    matcher->match(descriptor_L, descriptor_R, dmatch12); //dst_L -> dst_R
    matcher->match(descriptor_R, descriptor_L, dmatch21); //dst_R -> dst_L

    //dst_L -> dst_R と dst_R -> dst_Lの結果が一致しているか検証
    int good_count = 0;
    for (int i = 0; i < (int)dmatch12.size(); i++)
    {
        cv::DMatch m12 = dmatch12[i];
        cv::DMatch m21 = dmatch21[m12.trainIdx];

        if (m21.trainIdx == m12.queryIdx)
        { // 一致しているものだけを抜粋
            dmatch.push_back(m12);
            match_point1.push_back(keypoints_L[dmatch12[i].queryIdx].pt);
            match_point2.push_back(keypoints_R[dmatch12[i].trainIdx].pt);
            good_count++;
        }
    }

    //ホモグラフィ行列推定
    const int MIN_MATCH_COUNT = 10;
    if (good_count > MIN_MATCH_COUNT)
    { //十分対応点が見つかるならば
        cv::Mat masks;
        cv::Mat H = cv::findHomography(match_point1, match_point2, masks, cv::RANSAC, 1. / ransaq);

        //RANSACで使われた対応点のみ抽出
        for (int i = 0; i < masks.rows; i++)
        {
            uchar *inliner = masks.ptr<uchar>(i);
            if (inliner[0] == 1)
            {
                Dmatch.push_back(dmatch[i]);
            }
        }
    }

    //インライアの対応点のみ表示
    cv::Mat cvframe;
    cv::drawMatches(dst_L, keypoints_L, dst_R, keypoints_R, Dmatch, cvframe);
    if (show_camera)
    {
        cv::imshow("cvframe", cvframe);
        cv::waitKey(1);
    }

    ////    matching 終了   ////

    ////    reconstruction 開始     ////
    size_t match_num = Dmatch.size();
    if (match_num > 5)
    { //５点アルゴリズムが行えるのに十分対応点があれば
        //カメラの内部パラメータ(チェッカーボードから求めた焦点距離と主点座標)
        cv::Mat cameraMatrix(3, 3, CV_32FC1);
        float fovx = 1346.49, fovy = 1363.85, u0 = 302.86, v0 = 291.23;
        cameraMatrix = (cv::Mat_<float>(3, 3) << fovx, 0.0, u0, 0.0, fovy, v0, 0.0, 0.0, 1.0);

        //対応付いた特徴点の取り出しと焦点距離1.0のときの座標に変換
        std::vector<cv::Point2d> p1, p2;
        for (size_t i = 0; i < match_num; i++)
        {                               //特徴点の数だけ処理
            cv::Mat ip(3, 1, CV_32FC1); //3×1のベクトル
            cv::Point2d p;

            /*ip.at<float>(0) = keypoints_L[Dmatch[i].queryIdx].pt.x;	//1枚目の画像の特徴点のx座標を取得
            ip.at<float>(1) = keypoints_L[Dmatch[i].queryIdx].pt.y;	//1枚目の画像の特徴点のy座標を取得
            ip.at<float>(2) = 1.0;	//同次座標用
            ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
            p.x = ip.at<float>(0);
            p.y = ip.at<float>(1);*/

            p.x = keypoints_L[Dmatch[i].queryIdx].pt.x;
            p.y = keypoints_L[Dmatch[i].queryIdx].pt.y;
            p1.push_back(p);

            /*ip.at<float>(0) = keypoints_R[Dmatch[i].trainIdx].pt.x;	//2枚目の画像の特徴点のx座標を取得
            ip.at<float>(1) = keypoints_R[Dmatch[i].trainIdx].pt.y;	//2枚目の画像の特徴点のy座標を取得
            ip.at<float>(2) = 1.0;
            ip = cameraMatrix.inv()*ip;	//カメラのキャリブレーション行列により画像平面からカメラ平面へ写像
            p.x = ip.at<float>(0);
            p.y = ip.at<float>(1);*/

            p.x = keypoints_R[Dmatch[i].queryIdx].pt.x;
            p.y = keypoints_R[Dmatch[i].queryIdx].pt.y;
            p2.push_back(p);

            //printf("keypoint_L x = %0.1f, y = %0.1f\n", keypoints_L[dmatch[i].queryIdx].pt.x, keypoints_L[dmatch[i].queryIdx].pt.y);
        }

        //三角測量のためのカメラの透視射影行列
        cv::Mat prjMat1(3, 4, CV_32FC1), prjMat2(3, 4, CV_32FC1), out_prjMat1(3, 4, CV_32FC1), out_prjMat2(3, 4, CV_32FC1);
        cv::Mat R1(3, 3, CV_64FC1), t1(3, 1, CV_64FC1), Rt1(3, 4, CV_32FC1), R2(3, 3, CV_64FC1), t2(3, 1, CV_64FC1), Rt2(3, 4, CV_32FC1);
        cv::Mat out_R1, out_R2, out_Q;
        const float deg = 0.0 * CV_PI / 180.; //[rad]
        const float distance = 70.0;          //[mm]
        cv::Mat k(5, 1, CV_64FC1);
        Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0の外部パラメータ
        R1 = cv::Mat::eye(3, 3, CV_64FC1);
        t1 = (cv::Mat_<double>(3, 1) << 0., 0., 0.);
        Rt2 = (cv::Mat_<float>(3, 4) << cos(deg), -sin(deg), 0., distance, sin(deg), cos(deg), 0., 0., 0., 0., 1., 0.);
        R2 = (cv::Mat_<double>(3, 3) << cos(deg), -sin(deg), 0., sin(deg), cos(deg), 0., 0., 0., 1.);
        t2 = (cv::Mat_<double>(3, 1) << distance, 0., 0.);
        k = (cv::Mat_<double>(8, 1) << -0.742924, 3.492499, -0.013896, 0.003662, 0.0);
        prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
        prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列
        cv::stereoRectify(cameraMatrix, k, cameraMatrix, k, frame_image_L.size(), R2.t(), -t2, out_R1, out_R2, out_prjMat1, out_prjMat2, out_Q);
        //printf("%0.2lf, %0.2lf, %0.2lf, %0.2lf\n%0.2lf, %0.2lf, %0.2lf, %0.2lf\n%0.2lf, %0.2lf, %0.2lf, %0.2lf\n", prjMat2.at<float>(0, 0),prjMat2.at<float>(0, 1),prjMat2.at<float>(0, 2),prjMat2.at<float>(0, 3), prjMat2.at<float>(1, 0),prjMat2.at<float>(1, 1),prjMat2.at<float>(1, 2),prjMat2.at<float>(1, 3), prjMat2.at<float>(2, 0),prjMat2.at<float>(2, 1),prjMat2.at<float>(2, 2),prjMat2.at<float>(2, 3));

        //三角測量
        cv::Mat point4D(4, match_num, CV_32FC1), point4D_(4, match_num, CV_32FC1);
        cv::Mat out_prjMat1_32, out_prjMat2_32;
        out_prjMat1.convertTo(out_prjMat1_32, CV_32FC1);
        out_prjMat2.convertTo(out_prjMat2_32, CV_32FC1);
        cv::triangulatePoints(prjMat1, prjMat2, p1, p2, point4D);
        cv::triangulatePoints(out_prjMat1_32, out_prjMat2_32, p1, p2, point4D_);
        //prjMat1 : 運動学で求めたカメラ空間上での動作前のカメラの位置・姿勢行列
        //prjMat2 : 運動学で求めたカメラ空間上での動作後のカメラの位置・姿勢行列
        //p1      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間にで表現したもの
        //p2      : 画像平面上の特徴点座標に内部パラメータ行列をかけてカメラ空間で表現したもの
        //point4D : 三角測量で求めた各特徴点座標をカメラ空間にて表現したもの

        //Homogeneus座標からEuclid座標系への変換
        cv::Mat point3D(match_num, 1, CV_64FC3), point3D_(match_num, 1, CV_64FC3);
        cv::convertPointsFromHomogeneous(point4D.t(), point3D);
        cv::convertPointsFromHomogeneous(point4D_.t(), point3D_);

        // RViz2用に並べ替え
        cv::Mat pointCloud(match_num, 1, CV_32FC3);
        cv::Mat pointCloud2(match_num, 1, CV_32FC3);
        for (size_t i = 0; i < match_num; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                pointCloud.at<cv::Vec3f>(i)[j] = point3D_.at<cv::Vec3d>(i)[j];
            }
            printf("point4D     #%zd = [%0.4lf %0.4lf %0.4lf %0.4lf]\n", i, point4D.at<float>(0, i), point4D.at<float>(1, i), point4D.at<float>(2, i), point4D.at<float>(3, i));
            printf("point4D_    #%zd = [%0.4lf %0.4lf %0.4lf %0.4lf]\n", i, point4D_.at<float>(0, i), point4D_.at<float>(1, i), point4D_.at<float>(2, i), point4D_.at<float>(3, i));
            printf("point3D     #%zd = [%0.4lf %0.4lf %0.4lf]\n", i, point3D.at<cv::Vec3d>(i, 0)[0], point3D.at<cv::Vec3d>(i, 0)[1], point3D.at<cv::Vec3d>(i, 0)[2]);
            printf("point3D_    #%zd = [%0.4lf %0.4lf %0.4lf]\n", i, point3D_.at<cv::Vec3d>(i, 0)[0], point3D_.at<cv::Vec3d>(i, 0)[1], point3D_.at<cv::Vec3d>(i, 0)[2]);
            printf("pointCloud  #%zd = [%0.4f %0.4f %0.4f]\n", i, pointCloud.at<cv::Vec3f>(i, 0)[0], pointCloud.at<cv::Vec3f>(i, 0)[1], pointCloud.at<cv::Vec3f>(i, 0)[2]);
        }

        //距離（ノルム）が大きすぎる値は除去
        int dist_count = 0;
        for (size_t i = 0; i < match_num; ++i)
        {
            if (pointCloud.at<cv::Vec3f>(i)[2] > 0)
            {
                for (size_t j = 0; j < 3; ++j)
                {
                    pointCloud2.at<cv::Vec3f>(i - dist_count, 0)[j] = pointCloud.at<cv::Vec3f>(i, 0)[j];
                }
                printf("pointCloud2 #%zd = [%0.4f %0.4f %0.4f]\n", i, pointCloud2.at<cv::Vec3f>(i - dist_count, 0)[0], pointCloud2.at<cv::Vec3f>(i - dist_count, 0)[1], pointCloud2.at<cv::Vec3f>(i - dist_count, 0)[2]);
            }
            else
            {
                dist_count++;
            }
        }
        ////    reconstruction 終了     ////

        //Publish Image
        RCLCPP_INFO(logger, "Publishing image #%s", msg_image_L->header.frame_id.c_str());
        auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
        auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
        convert_frame_to_message(cvframe, atoi(msg_image_L->header.frame_id.c_str()), *msg_pub); //cv → msg
        convert_pointcloud_to_PCL(pointCloud2, *msg_cloud_pub, dist_count);
        //convert_pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);
        pub->publish(std::move(msg_pub));
        pub_pointcloud->publish(std::move(msg_cloud_pub));
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
    size_t ransaq = 20;
    size_t feature = 0;
    size_t match = 0;
    size_t prjMat = 1;

    std::string topic_sub_image_L("stereo_image_L");
    std::string topic_sub_image_R("stereo_image_R");
    std::string topic_pub("feature_point");
    std::string topic_pub_pointcloud("pointcloud");

    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Configure demo parameters with command line options.
    if (!parse_command_options(
            argc, argv, &depth, &reliability_policy, &history_policy, &show_camera, &ransaq, &feature, &match, &prjMat))
    {
        return 0;
    }

    if (show_camera)
    {
        // Initialize an OpenCV named window called "cvframe".
        cv::namedWindow("cvframe", cv::WINDOW_AUTOSIZE);
    }
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("stereo_reconstruction");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub.c_str());
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_pointcloud.c_str());

    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);                             // Create the image publisher with our custom QoS profile.
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_L(node.get(), topic_sub_image_L);
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_R(node.get(), topic_sub_image_R);
    message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync(image_sub_L, image_sub_R, 10);
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, ransaq, feature, match, node_logger, pub, pub_pointcloud));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}