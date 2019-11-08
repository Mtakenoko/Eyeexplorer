#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include <ktl.h>

#include "../include/arm/arm_fk.h"
#include "../include/arm/arm.h"

#define ENC_OFFSET_FILE  "enc_offset.dat"  ///< エンコーダオフセットファイルのパス


using namespace std::chrono_literals;

//アーム初期
PassiveArm passivearm;

bool ReadEncoder::transformRotMatToQuaternion(
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
void ReadEncoder::QuaternionToEulerAngles(double q0, double q1, double q2, double q3,
                             double& roll, double& pitch, double& yaw)
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
void ReadEncoder::EncoderOffset(Ktl::Vector<ADOF> &offset){
  std::ifstream ifs(ENC_OFFSET_FILE);

  for( int i = 0; i < ADOF; i++ )
    ifs >> offset[i];

  ifs.close();
  offset *= 1.0 / DEG; //deg -> rad
}

ReadEncoder::ReadEncoder()
: Node("arm_fk"), count_(0){

    //Initialize
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    std::string topic_sub("ts01_encoder");   
    std::string topic_pub_tip("arm_trans");

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize a ROS node.
    rclcpp::Logger node_logger = this->get_logger();

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);
    
    Ktl::Vector<ADOF> qoffset;
    ReadEncoder::EncoderOffset(qoffset);
    RCLCPP_INFO(this->get_logger(), "arm_fk node ");

    auto callback = [this, &qoffset](const std_msgs::msg::Float32MultiArray::SharedPtr sub_msg) -> void{
        for(int i=0; i<ADOF; i++){
            passivearm.q[i] = sub_msg->data[i] - qoffset[i];
        }
        passivearm.forward_kinematics();
    };  
    auto publish_callback = [this]() -> void{     
        //送信するメッセージ
        pub_msg_ = std::make_shared<geometry_msgs::msg::Transform>();
        tf2_ros::StaticTransformBroadcaster broadcaster(this);
        geometry_msgs::msg::TransformStamped msg;

        //位置・姿勢計算
        Ktl::Matrix<3, 3> Escope = passivearm.Tw *
                                    Ktl::Matrix<3,3>(Ktl::Y,  180.0 / DEG) *
                                    Ktl::Matrix<3,3>(Ktl::Z, -90.0 / DEG);
        Ktl::Matrix<3, 3> endoscope_pose = passivearm.Rr() * Escope;  // 内視鏡姿勢行列
        Ktl::Vector<3> n = endoscope_pose.column(2);  // 内視鏡の向き
        Ktl::Vector<3> Ptip = passivearm.Pr() + ENDOSCOPE_LENGTH * n;
        
        //並進成分
        pub_msg_->translation.x = Ptip[0];
        pub_msg_->translation.y = Ptip[1];
        pub_msg_->translation.z = Ptip[2];
        msg.transform.translation.x = Ptip[0];
        msg.transform.translation.y = Ptip[1];
        msg.transform.translation.z = Ptip[2];        

        //回転行列
        float qx, qy, qz, qw; 
        transformRotMatToQuaternion(qx, qy, qz, qw,
                        endoscope_pose[0][0], endoscope_pose[1][0], endoscope_pose[2][0],
                        endoscope_pose[0][1], endoscope_pose[1][1], endoscope_pose[2][1],
                        endoscope_pose[0][2], endoscope_pose[1][2], endoscope_pose[2][2]);
        pub_msg_->rotation.x = qx;
        pub_msg_->rotation.y = qy;
        pub_msg_->rotation.z = qz;
        pub_msg_->rotation.w = qw;
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
        QuaternionToEulerAngles(qx, qy,qz, qw, rall, pitch, yaw);
        static int count = 0;
        count++;
        if(count % 10 == 0){
            RCLCPP_INFO(this->get_logger(), "Ptip = [%0.2f %0.2f %0.2f] Prot = [%0.2f %0.2f %0.2f]", Ptip[0], Ptip[1], Ptip[2], rall, pitch, yaw);
        }
        
        //Publish
        publisher_->publish(*pub_msg_);
        broadcaster.sendTransform(msg);
    };
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Transform>(topic_pub_tip,qos);
    subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(topic_sub,qos,callback);
    timer_ = this->create_wall_timer(1ms, publish_callback);
}

