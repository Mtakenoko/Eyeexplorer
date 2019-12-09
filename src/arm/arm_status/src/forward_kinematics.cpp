#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/clock.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <ktl.h>

#include "../include/arm/encoder.h"
#include "../include/arm/arm.h"

#define ENC_OFFSET_FILE  "enc_offset.dat"  ///< エンコーダオフセットファイルのパス

using namespace std::chrono_literals;

ReadEncoder readencoder;
PassiveArm passivearm;
geometry_msgs::msg::Transform tip_msg;
sensor_msgs::msg::JointState q_msg;
geometry_msgs::msg::TransformStamped tf_msg;


void forward_kinematics(const sensor_msgs::msg::JointState::SharedPtr sub_msg, Ktl::Vector<ADOF> qoffset,
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Transform>> pub_tip, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> pub_q, rclcpp::Clock::SharedPtr clock,
    rclcpp::Logger logger, std::shared_ptr<rclcpp::Node> node){
    //送信するメッセージ
    tf2_ros::StaticTransformBroadcaster broadcaster(node);
    
    //エンコーダーの値を読んで運動学を解く
    for(int i=0; i<ADOF; i++){
        passivearm.q[i] = sub_msg->position[i] - qoffset[i];
        
    }
    q_msg.position.resize(ADOF+1);
    q_msg.position[0] = passivearm.q[0];
    q_msg.position[1] = passivearm.q[1];
    q_msg.position[2] = passivearm.q[2];
    q_msg.position[3] = PI / 4 - (passivearm.q[1] + passivearm.q[2]);
    q_msg.position[4] = passivearm.q[3];
    q_msg.position[5] = passivearm.q[4];
    
    q_msg.header.stamp = clock->now();
    passivearm.forward_kinematics();

    //位置・姿勢計算
    Ktl::Matrix<3, 3> Escope = passivearm.Tw *
                                Ktl::Matrix<3,3>(Ktl::Y,  180.0 / DEG) *
                                Ktl::Matrix<3,3>(Ktl::Z, -90.0 / DEG);
    Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;  // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);  // 内視鏡の向き
    Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;    
    
    //並進成分
    tip_msg.translation.x = Ptip[0];
    tip_msg.translation.y = Ptip[1];
    tip_msg.translation.z = Ptip[2];
    tf_msg.transform.translation.x = Ptip[0] / 1000.;
    tf_msg.transform.translation.y = Ptip[1] / 1000.;
    tf_msg.transform.translation.z = Ptip[2] / 1000.;        

    //回転行列
    float qx, qy, qz, qw; 
    readencoder.transformRotMatToQuaternion(qx, qy, qz, qw,
                    endoscope_pose[0][0], endoscope_pose[1][0], endoscope_pose[2][0],
                    endoscope_pose[0][1], endoscope_pose[1][1], endoscope_pose[2][1],
                    endoscope_pose[0][2], endoscope_pose[1][2], endoscope_pose[2][2]);
    tip_msg.rotation.x = qx;
    tip_msg.rotation.y = qy;
    tip_msg.rotation.z = qz;
    tip_msg.rotation.w = qw;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;

    //
    //msg.header.stamp = ;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "arm_tip";
    
    //表示
    double rall, pitch, yaw;
    readencoder.QuaternionToEulerAngles(qx, qy, qz, qw, rall, pitch, yaw);
    
    static int count = 0;
    count++;
    if(count % 10 == 0){
        RCLCPP_INFO(logger, "t = [%0.2f %0.2f %0.2f] R = [%0.2f %0.2f %0.2f]", Ptip[0], Ptip[1], Ptip[2], rall, pitch, yaw);
    }
    
    //Publish
    pub_tip->publish(tip_msg);
    pub_q->publish(q_msg);
    broadcaster.sendTransform(tf_msg);
}

int main(int argc, char * argv[]){
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("ts01_encoder");   
    std::string topic_pub_tip("arm_trans");
    std::string topic_pub_q("joint_states");

    //エンコーダーオフセット
    Ktl::Vector<ADOF> qoffset;
    readencoder.EncoderOffset(qoffset);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("forward_kinematics");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
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
    auto pub_q = node->create_publisher<sensor_msgs::msg::JointState>(topic_pub_q, 10); // Create the image publisher with our custom QoS profile.

    //setting q_msg
    //q_msg.name.resize(6);
    q_msg.name.push_back("arm_joint1");
    q_msg.name.push_back("arm_joint2");
    q_msg.name.push_back("arm_joint3");
    q_msg.name.push_back("arm_joint_horiz");
    q_msg.name.push_back("arm_joint4");
    q_msg.name.push_back("arm_joint5");
    //q_msg.position.resize(6);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);
    q_msg.position.push_back(0.0);

    auto callback = [qoffset, pub_tip, pub_q, clock, &node](const sensor_msgs::msg::JointState::SharedPtr msg_sub){
        forward_kinematics(msg_sub, qoffset, pub_tip, pub_q, clock, node->get_logger(), node);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(topic_sub, qos, callback);  // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
