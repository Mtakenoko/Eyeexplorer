#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <ktl.h>

#include "../include/arm/encoder.h"
#include "../include/arm/arm.h"
#include "/home/takeyama/workspace/htl/opencv/transform.hpp"

using namespace std::chrono_literals;

ReadEncoder readencoder;
PassiveArm passivearm;
geometry_msgs::msg::Transform tip_msg;
sensor_msgs::msg::JointState q_msg;
geometry_msgs::msg::TransformStamped tf_msg;

void forward_kinematics(const sensor_msgs::msg::JointState::SharedPtr sub_msg,
                        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Transform>> pub_tip, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> pub_q, rclcpp::Clock::SharedPtr clock,
                        std::shared_ptr<rclcpp::Node> node)
{
    //エンコーダの値の向きをあわせる
    Ktl::Vector<ADOF> enc_pos;
    enc_pos[0] = sub_msg->position[0];
    enc_pos[1] = sub_msg->position[1];
    enc_pos[2] = sub_msg->position[2];
    enc_pos[3] = sub_msg->position[3];
    enc_pos[4] = sub_msg->position[4];

    //エンコーダーの値を読んで運動学を解く
    for (int i = 0; i < ADOF; i++)
    {
        passivearm.q[i] = enc_pos[i] + readencoder.offset[i];
    }
    passivearm.forward_kinematics();

    // q_msgにデータ入力
    q_msg.position[0] = passivearm.q[0];
    q_msg.position[1] = passivearm.q[1];
    q_msg.position[2] = -passivearm.q[1];
    q_msg.position[3] = passivearm.q[2];
    q_msg.position[4] = -passivearm.q[2];
    q_msg.position[5] = passivearm.q[3];
    q_msg.position[6] = passivearm.q[4];

    q_msg.header.stamp = clock->now();

    //位置・姿勢計算
    // 内視鏡針部の付け根までの運動学
    Ktl::Matrix<3, 3> Escope = Ktl::Matrix<3, 3>(Ktl::Y, 180.0 / DEG) *
                               Ktl::Matrix<3, 3>(Ktl::Z, 0.0 / DEG);  //現状は内視鏡の姿勢はx軸が視線方向なので画像座標と等しく（z正方向が視線方向）するための回転行列?
    Ktl::Matrix<3, 3> endoscope_root_pose = passivearm.Rr() * Escope; // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_root_pose.column(2);                 // 内視鏡の向き
    Ktl::Vector<3> P_root = passivearm.Pr() + ENDOSCOPE_ROOT * n;

    // 内視鏡針部のたわみについて
    // 梁の曲げモデルを考える。x軸回りの曲げ回転とその方向を選択するz方向の回転の2つで成り立つ
    Ktl::Matrix<3, 3> Rot_r_e = Ktl::Matrix<3, 3>(Ktl::Z, DEFLECTION_DERECT / DEG);                   // 針部の曲がる平面（力方向に水平な平面）への座標変換行列（内視鏡姿勢座標からみた）
    Ktl::Matrix<3, 3> Rot_deflect_r = Ktl::Matrix<3, 3>(Ktl::X, -M_EI * ENDOSCOPE_NEEDLE);            // 針部の曲げを表す回転行列（力に水平な面から見た）
    Ktl::Vector<3> needle_r(0., M_EI * ENDOSCOPE_NEEDLE * ENDOSCOPE_NEEDLE / 2., ENDOSCOPE_NEEDLE);   // 内視鏡根本から内視鏡先端までのベクトル（力に水平な面から見た）
    Ktl::Matrix<3, 3> endoscope_pose = endoscope_root_pose * Rot_r_e * Rot_deflect_r * Rot_r_e.inv(); // 絶対座標系から見た、内視鏡先端の座標系
    Ktl::Vector<3> Ptip = P_root + endoscope_root_pose * Rot_r_e * needle_r;

    //並進成分
    tip_msg.translation.x = Ptip[0];
    tip_msg.translation.y = Ptip[1];
    tip_msg.translation.z = Ptip[2];

    //回転行列
    float qx, qy, qz, qw;
    htl::Transform::RotMatToQuaternion(&qx, &qy, &qz, &qw,
                                 (float)endoscope_pose[0][0], (float)endoscope_pose[0][1], (float)endoscope_pose[0][2],
                                 (float)endoscope_pose[1][0], (float)endoscope_pose[1][1], (float)endoscope_pose[1][2],
                                 (float)endoscope_pose[2][0], (float)endoscope_pose[2][1], (float)endoscope_pose[2][2]);
    tip_msg.rotation.x = qx;
    tip_msg.rotation.y = qy;
    tip_msg.rotation.z = qz;
    tip_msg.rotation.w = qw;

    //表示
    double rall, pitch, yaw;
    htl::Transform::QuaternionToEulerAngles((double)qx, (double)qy, (double)qz, (double)qw, rall, pitch, yaw);

    static int count = 0;
    count++;
    if (count % 10 == 0)
    {
        RCLCPP_INFO(node->get_logger(), "zim: t = [%0.2f %0.2f %0.2f], R = [%0.2f %0.2f %0.2f]", passivearm.Pr()[0], passivearm.Pr()[1], passivearm.Pr()[2], rall, pitch, yaw);
        RCLCPP_INFO(node->get_logger(), "tip: t = [%0.2f %0.2f %0.2f], R = [%0.2f %0.2f %0.2f]", Ptip[0], Ptip[1], Ptip[2], rall, pitch, yaw);
        RCLCPP_INFO(node->get_logger(), "root: t = [%0.2f %0.2f %0.2f], R = [%0.2f %0.2f %0.2f]", P_root[0], P_root[1], P_root[2], rall, pitch, yaw);
        // printf("q = [%lf %lf %lf %lf %lf]\n", passivearm.q[0], passivearm.q[1], passivearm.q[2], passivearm.q[3], passivearm.q[4]);
        // printf("q = [%lf %lf %lf %lf %lf]\n", enc_pos[0], enc_pos[1], enc_pos[2], enc_pos[3], enc_pos[4]);
    }

    //Publish
    pub_tip->publish(tip_msg);
    // pub_q->publish(q_msg);

    //ここからtf
    //送信するメッセージ
    tf2_ros::StaticTransformBroadcaster broadcaster(node);

    tf_msg.transform.translation.x = Ptip[0] / 1000.;
    tf_msg.transform.translation.y = Ptip[1] / 1000.;
    tf_msg.transform.translation.z = Ptip[2] / 1000.;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "arm_tip";
    broadcaster.sendTransform(tf_msg);
}

int main(int argc, char *argv[])
{
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("/ts01/encoder");
    std::string topic_pub_tip("arm_trans");
    std::string topic_pub_q("joint_states");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("forward_kinematics");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.readencoder.Getoffset()
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //時間管理
    rclcpp::TimeSource ts(node);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_tip.c_str());
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_q.c_str());
    auto pub_tip = node->create_publisher<geometry_msgs::msg::Transform>(topic_pub_tip, qos); // Create the image publisher with our custom QoS profile.
    auto pub_q = node->create_publisher<sensor_msgs::msg::JointState>(topic_pub_q, 10);       // Create the image publisher with our custom QoS profile.

    //setting q_msg
    q_msg.name.push_back("arm_joint1");
    q_msg.name.push_back("arm_joint2");
    q_msg.name.push_back("arm_joint_horiz");
    q_msg.name.push_back("arm_joint3");
    q_msg.name.push_back("arm_joint_horiz2");
    q_msg.name.push_back("arm_joint4");
    q_msg.name.push_back("arm_joint5");
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);

    //エンコーダのオフセット設定
    readencoder.SetOffset();
    readencoder.ReadCalibOffsetdat();

    auto callback = [pub_tip, pub_q, clock, &node](const sensor_msgs::msg::JointState::SharedPtr msg_sub) {
        forward_kinematics(msg_sub, pub_tip, pub_q, clock, node);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(topic_sub, qos, callback); // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
