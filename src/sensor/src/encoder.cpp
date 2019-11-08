#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video.hpp"

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <ktl.h>
#include <ktl/rttask.h>
#include <libts01.h>

#include "../include/sensor/encoder.h"
#include "my_messages/srv/calc_two_floats.hpp"

////////計測用パラメータなど設定/////////
# define ADOF 5;
//アブソエンコーダのビット数の設定
static const double ARM1_BIT[5] = {
    20, 19, 19, 18, 18
};    
//分解能（パルス数）
unsigned int res(int n)
{
  //16進数0xffffffff　=>　2進数2＾32
  //右シフトで分解能に変換
  //例（18bitのエンコーダ）
  //11...1=>32個
  //右に（32-18）bitシフト
  //00..0->14個　11..1->18個　=>2の18乗
  return 0xffffffff >> (32 - n);
}

//「0〜max」から「-max/2〜max/2」に変更
int shift_range(unsigned int val, unsigned int range)
{
  if (val < range / 2) //+30000)
    return (int)val;
  else
    return (int)val - (int)range;
}
///////////////////////////

using namespace std::chrono_literals;

bool check_ts01(const std_msgs::msg::Bool::SharedPtr msg, rclcpp::Logger logger){
  if(msg->data){
    RCLCPP_INFO(logger, "Received TS01 is opened!");
    return true;
  }else{
    return false;
  }
}

void count_encoder(rclcpp::Logger node_logger){
  TS01InputData input;
  //エンコーダのカウント
  int enc[5];
  enc[0] = shift_range(input.ssi[0] >> 1, 0x0000fffff); //-2^19~2^19
  enc[1] = shift_range(input.ssi[1] >> 1, 0x00007ffff); //-2^17~2^17
  enc[2] = shift_range(input.ssi[2] >> 1, 0x00007ffff); 
  enc[3] = shift_range(input.ssi[3] >> 1, 0x00003ffff); //
  enc[4] = shift_range(input.ssi[4] >> 1, 0x00003ffff); //
  for(int i=0; i<5;i++){
    RCLCPP_INFO(node_logger, "Encoder #%zd:enc = %zd", i, enc[i]);
  }
}
void ReadEncoder::handleService_(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<my_messages::srv::CalcTwoFloats::Request> request,
  const std::shared_ptr<my_messages::srv::CalcTwoFloats::Response> response
){
  (void)request_header;
  RCLCPP_INFO(this->get_logger(),"srv.request:%lf,%lf", request->a, request->b);
  response->sum = request->a + request->b;
  response->diff = request->a - request->b;
}

ReadEncoder::ReadEncoder ()
: Node("ReadEncoder"), count_(0){
  // Initialize default demo parameters
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

  // Set quality of service profile based on command line options.
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  static bool ts01_status = false;

  publisher_ = this->create_publisher<std_msgs::msg::String>("ReadEncoder", qos);
  count_encoder(this->get_logger());

  srv_ = this->create_service<my_messages::srv::CalcTwoFloats>(
    "srv_test",
    std::bind(&ReadEncoder::handleService_, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
  );
}