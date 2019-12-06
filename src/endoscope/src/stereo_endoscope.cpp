#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <ktl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/stereo.hpp>

#include <rclcpp/rclcpp.hpp>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>

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
    if(frame2 < 10 || frame_span < 15){
        return;
    }

    //Subscribe
    cv::Mat frame_image(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);

    if (msg_image->encoding == "rgb8") {
    cv::cvtColor(frame_image, frame_image, cv::COLOR_RGB2BGR);
    }

    //アーム姿勢をcv::Mat形式に変換
    static cv::Mat x1(3, 1, CV_32FC1), x2(3, 1, CV_32FC1);
    static cv::Mat r1(3, 3, CV_32FC1), r2(3, 3, CV_32FC1);
    x2 = arm_trans.clone();
    r2 = arm_rot.clone();

    static cv::Mat pre_frame_image, pre_image, current_image;
    static int loop_count = 0; loop_count++;
    if(loop_count < 3){
        x1 = x2.clone();
        r1 = r2.clone();
        frame1 = frame2;
        pre_frame_image = frame_image.clone();
        return;
    }

    //アームの運動学で求めた位置・姿勢の変化量
    cv::Mat r_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
    r_arm = r2 * r1.inv();
    t_arm = r1 * (x2 - x1);

    //1枚目の画像を回転させて二枚目の画像と

    //disparity_mapを作成する
    cv::Mat disparity_data, disparity_map, dis_data, dis_map;
    double min, max;

    //グレイスケールへ変換
    printf("typergb : %d\n", pre_frame_image.type());
    cv::cvtColor(pre_frame_image, pre_image, CV_BGR2GRAY);
    cv::cvtColor(frame_image, current_image, CV_BGR2GRAY);
    printf("typegry : %d\n", pre_image.type());
    
    //StereoBMデフォルト設定
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();
    sbm->setNumDisparities(160);
    sbm->setBlockSize(21);
    sbm->compute(pre_image, current_image, disparity_data);
    cv::minMaxLoc(disparity_data, &min, &max);
    disparity_data.convertTo(disparity_map, CV_8UC1, 255/(max-min), -255*min/(max-min));
    cv::imshow("StereoBM画像", disparity_map);
    cv::imshow("元画像",frame_image);

    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,    //int minDisparity
                                        96,     //int numDisparities
                                        5,      //int SADWindowSize
                                        600,    //int P1 = 0
                                        2400,   //int P2 = 0
                                        20,     //int disp12MaxDiff = 0
                                        16,     //int preFilterCap = 0
                                        1,      //int uniquenessRatio = 0
                                        100,    //int speckleWindowSize = 0
                                        20,     //int speckleRange = 0
                                        true);  //bool fullDP = false

    sgbm->compute(pre_image, current_image, dis_data);
    cv::minMaxLoc(dis_data, &min, &max);
    dis_data.convertTo(dis_map, CV_8UC1, 255/(max-min), -255*min/(max-min));
    cv::imshow("StereoSGBM画像", dis_map);
    cv::waitKey(1);
 
    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    //auto msg_cloud_pub = std::make_shared<sensor_msgs::msg::PointCloud2>();
    convert_frame_to_message(frame_image, atoi(msg_image->header.frame_id.c_str()), *msg_pub);  //cv → msg
    pub->publish(std::move(msg_pub)); 

    // 1つめのフレームに代入
    x1 = x2.clone();
    r1 = r2.clone();
    frame1 = frame2;
    pre_frame_image = frame_image.clone();
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

    std::string topic_sub("image");   
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