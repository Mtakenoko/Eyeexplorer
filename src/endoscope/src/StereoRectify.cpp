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

void callback(const std::shared_ptr<const sensor_msgs::msg::Image> &msg_image, const std::shared_ptr<const sensor_msgs::msg::Image> &msg_mask, bool show_camera, cv::Mat arm_trans, cv::Mat arm_rot, rclcpp::Logger logger, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> pub, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_pointcloud, std::shared_ptr<rclcpp::Node> node)
{
    RCLCPP_INFO(logger, "Received image #%s", msg_image->header.frame_id.c_str());

    //frame間隔をあける
    int frame_span;
    static int frame1 = 0, frame2;
    frame2 = atoi(msg_image->header.frame_id.c_str());
    frame_span = frame2 - frame1;
    if (frame2 < 10 || frame_span < 10)
    {
        return;
    }

    //Subscribe
    static cv::Mat frame_image_1;
    cv::Mat frame_image_2(msg_image->height, msg_image->width, encoding2mat_type(msg_image->encoding), const_cast<unsigned char *>(msg_image->data.data()), msg_image->step);
    
    if (msg_image->encoding == "rgb8")
    {
        cv::cvtColor(frame_image_2, frame_image_2, cv::COLOR_RGB2BGR);
    }
    //アーム姿勢をcv::Mat形式に変換
    static cv::Mat x1(3, 1, CV_32FC1), x2(3, 1, CV_32FC1);
    static cv::Mat r1(3, 3, CV_32FC1), r2(3, 3, CV_32FC1);
    x2 = arm_trans.clone();
    r2 = arm_rot.clone();

    static int loop_count = 0;
    loop_count++;
    if (loop_count < 3)
    {
        x1 = x2.clone();
        r1 = r2.clone();
        frame1 = frame2;
        frame_image_1 = frame_image_2.clone();
        return;
    }
    ////    detection 終了  ////

    ////カメラのステレオ平行化
    //カメラの内部パラメータ
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    double fovx = 364.283456, fovy = 366.617877, u0 = 159.699646, v0 = 155.864578;
    cameraMatrix = (cv::Mat_<double>(3, 3) << fovx, 0.0, u0, 0.0, fovy, v0, 0.0, 0.0, 1.0);
    //カメラの外部パラメータ
    cv::Mat r_arm(3, 3, CV_32FC1), t_arm(3, 1, CV_32FC1);
    cv::Mat Rt1(3, 4, CV_32FC1), Rt2(3, 4, CV_32FC1);
    r_arm = r2 * r1.inv();
    t_arm = r1 * (x2 - x1);
    Rt1 = cv::Mat::eye(3, 4, CV_32FC1); //片方は回転、並進ともに0の外部パラメータ
    Rt2 = (cv::Mat_<float>(3, 4) << r_arm.at<float>(0, 0), r_arm.at<float>(0, 1), r_arm.at<float>(0, 2), t_arm.at<float>(0), r_arm.at<float>(1, 0), r_arm.at<float>(1, 1), r_arm.at<float>(1, 2), t_arm.at<float>(1), r_arm.at<float>(2, 0), r_arm.at<float>(2, 1), r_arm.at<float>(2, 2), t_arm.at<float>(2));
    //カメラの歪み補正行列
    cv::Mat k(8, 1, CV_32FC1);
    //カメラの透視射影行列
    cv::Mat prjMat1(3, 4, CV_32FC1), prjMat2(3, 4, CV_32FC1);
    prjMat1 = cameraMatrix * Rt1; //内部パラメータと外部パラメータをかけて透視射影行列
    prjMat2 = cameraMatrix * Rt2; //内部パラメータと外部パラメータをかけて透視射影行列

    //キャリブレーション済みステレオの，それぞれのカメラの平行化変換
    cv::Mat out_R1, out_R2;
    cv::Mat out_P1, out_P2;
    cv::Mat out_Q;
    double out_alpha;
    cv::Size newimage_size;
    cv::Rect out_ROI1, out_ROI2;
    cv::stereoRectify(cameraMatrix, k, cameraMatrix, k, frame_image_2.size(),
                      r_arm, t_arm, out_R1, out_R2, out_P1, out_P2, out_Q,
                      CV_CALIB_ZERO_DISPARITY, out_alpha, newimage_size, &out_ROI1, &out_ROI2);

    //平行ステレオでのステレオビジョン
    cv::Mat frame_image_L, frame_image_R, frame_image_R_;
    frame_image_L = frame_image_1.clone();
    cv::projectPoints(frame_image_R, out_R2, 0, cameraMatrix, k, frame_image_R_);

    //disparity_mapを作成する
    cv::Mat disparity_data, disparity_map, dis_data, dis_map;
    const double min = -16;
    const double max = 2544;

    //グレイスケールへ変換
    cv::Mat gr_image_L, gr_image_R;
    cv::cvtColor(frame_image_L, gr_image_L, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame_image_R, gr_image_R, cv::COLOR_BGR2GRAY);

    //StereoBMデフォルト設定
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();
    sbm->setNumDisparities(160);
    sbm->setBlockSize(11);
    sbm->compute(gr_image_L, gr_image_R, disparity_data);
    //cv::minMaxLoc(disparity_data, &min, &max);
    disparity_data.convertTo(disparity_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));

    //SGBM
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0,      //int minDisparity
        160,    //int numDisparities
        11,     //int SADWindowSize
        0,      //int P1 = 0
        0,      //int P2 = 0
        0,      //int disp12MaxDiff = 0
        0,      //int preFilterCap = 0
        0,      //int uniquenessRatio = 0
        100,    //int speckleWindowSize = 0
        1,      //int speckleRange = 0
        false); //bool fullDP = false

    sgbm->compute(gr_image_L, gr_image_R, dis_data);
    //cv::minMaxLoc(dis_data, &min, &max);
    dis_data.convertTo(dis_map, CV_8UC1, 255 / (max - min), -255 * min / (max - min));
    cv::Mat show_map;
    cv::hconcat(disparity_map, dis_map, show_map);
        cv::imshow("stereo_vision_map", show_map);
        cv::waitKey(1);
    if (show_camera)
    {
    }
    //Publish Image
    RCLCPP_INFO(logger, "Publishing image #%s", msg_image->header.frame_id.c_str());
    auto msg_pub = std::make_unique<sensor_msgs::msg::Image>();
    //auto msg_cloud_pub = std::make_unique<sensor_msgs::msg::PointCloud2>();
    convert_frame_to_message(show_map, atoi(msg_image->header.frame_id.c_str()), *msg_pub); //cv → msg
    //convert_pointcloud_to_PCL(pointCloud2, *msg_cloud_pub, dist_count);
    //convert_pointcloud_to_PCL(pointCloud, *msg_cloud_pub, 0);

    pub->publish(std::move(msg_pub));
    //pub_pointcloud->publish(std::move(msg_cloud_pub));

    // 1つめのフレームに代入
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

    if (show_camera)
    {
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

    auto pub = node->create_publisher<sensor_msgs::msg::Image>(topic_pub, qos);                             // Create the image publisher with our custom QoS profile.
    auto pub_pointcloud = node->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_pointcloud, qos); // Create the image publisher with our custom QoS profile.

    //グローバル座標とカメラ座標間の座標変換行列
    cv::Mat arm_trans(3, 1, CV_32FC1);
    cv::Mat arm_rot(3, 3, CV_32FC1);
    auto callback_arm_trans = [&arm_trans, &arm_rot, &node](const geometry_msgs::msg::Transform::SharedPtr msg_sub) {
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
    sync.registerCallback(std::bind(&callback, std::placeholders::_1, std::placeholders::_2, show_camera, arm_trans, arm_rot, node_logger, pub, pub_pointcloud, node));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}