#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

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

void compensation(Ktl::Vector<3> Pgim, Ktl::Vector<3> n, Ktl::Vector<3> &Pest){
    const double l_RCM = 50.0;
    Pest = Pgim + l_RCM * (n - n) + ENDOSCOPE_LENGTH * n;
}

void forward_kinematics(const sensor_msgs::msg::JointState::SharedPtr sub_msg, Ktl::Vector<ADOF> qoffset,
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Transform>> pub, rclcpp::Logger logger, std::shared_ptr<rclcpp::Node> node){
    //エンコーダーの値を読んで運動学を解く
    for(int i=0; i<ADOF; i++){
            passivearm.q[i] = sub_msg->position[i] - qoffset[i];
    }
    passivearm.forward_kinematics();
    
    //送信するメッセージ
    auto pub_msg = std::make_unique<geometry_msgs::msg::Transform>();
    tf2_ros::StaticTransformBroadcaster broadcaster(node);
    geometry_msgs::msg::TransformStamped msg;

    //位置・姿勢計算
    Ktl::Matrix<3, 3> Escope = passivearm.Tw *
                                Ktl::Matrix<3,3>(Ktl::Y,  180.0 / DEG) *
                                Ktl::Matrix<3,3>(Ktl::Z, -90.0 / DEG);
    Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;  // 内視鏡姿勢行列
    Ktl::Vector<3> n = endoscope_pose.column(2);  // 内視鏡の向き
    Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;
    
    //姿勢変化を利用して補正を行う
    Ktl::Vector<3> Pest;
    compensation(passivearm.Pr(), n, Pest);

    //並進成分
    pub_msg->translation.x = Ptip[0];
    pub_msg->translation.y = Ptip[1];
    pub_msg->translation.z = Ptip[2];
    msg.transform.translation.x = Ptip[0];
    msg.transform.translation.y = Ptip[1];
    msg.transform.translation.z = Ptip[2];        

    //回転行列
    float qx, qy, qz, qw; 
    readencoder.transformRotMatToQuaternion(qx, qy, qz, qw,
                    endoscope_pose[0][0], endoscope_pose[1][0], endoscope_pose[2][0],
                    endoscope_pose[0][1], endoscope_pose[1][1], endoscope_pose[2][1],
                    endoscope_pose[0][2], endoscope_pose[1][2], endoscope_pose[2][2]);
    pub_msg->rotation.x = qx;
    pub_msg->rotation.y = qy;
    pub_msg->rotation.z = qz;
    pub_msg->rotation.w = qw;
    msg.transform.rotation.x = qx;
    msg.transform.rotation.y = qy;
    msg.transform.rotation.z = qz;
    msg.transform.rotation.w = qw;

    //
    //msg.header.stamp = ;
    msg.header.frame_id = "map";
    msg.child_frame_id = "arm_tip";
    
    //表示
    double rall, pitch, yaw;
    readencoder.QuaternionToEulerAngles(qx, qy,qz, qw, rall, pitch, yaw);
    static int count = 0;
    count++;
    if(count % 10 == 0){
        RCLCPP_INFO(logger, "Ptip = [%0.2f %0.2f %0.2f] Prot = [%0.2f %0.2f %0.2f]", Ptip[0], Ptip[1], Ptip[2], rall, pitch, yaw);
    }
    
    //Publish
    pub->publish(std::move(pub_msg));
    broadcaster.sendTransform(msg);
}

int main(int argc, char * argv[]){
    //Initialize
    rclcpp::init(argc, argv);

    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("ts01_encoder");   
    std::string topic_pub_tip("arm_trans");

    //エンコーダーオフセット
    Ktl::Vector<ADOF> qoffset;
    readencoder.EncoderOffset(qoffset);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // Initialize a ROS node.
    auto node = rclcpp::Node::make_shared("arm_forward_kinematics");
    rclcpp::Logger node_logger = node->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //Set QoS to Publish
    RCLCPP_INFO(node->get_logger(), "Publishing data on topic '%s'", topic_pub_tip.c_str());
    auto pub = node->create_publisher<geometry_msgs::msg::Transform>(topic_pub_tip, qos); // Create the image publisher with our custom QoS profile.

    auto callback = [qoffset, pub, &node](const sensor_msgs::msg::JointState::SharedPtr msg_sub){
        forward_kinematics(msg_sub, qoffset, pub, node->get_logger(), node);
    };

    //Set QoS to Subscribe
    RCLCPP_INFO(node->get_logger(), "Subscribing to topic '%s'", topic_sub.c_str());
    auto sub = node->create_subscription<std_msgs::msg::Float32MultiArray>(topic_sub, qos, callback);  // Initialize a subscriber that will receive the ROS Image message to be displayed.

    rclcpp::spin(node);
    rclcpp::shutdown();    

}