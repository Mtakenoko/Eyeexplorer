#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <ktl.h>
#include <libts01.h>

using namespace std::chrono_literals;

TS01 ts01;
TS01InputData input;
TS01OutputData output;

#define ADOF 5
#define ENC_OFFSET_FILE "enc_offset.dat"

//アブソエンコーダのビット数の設定
static const short ARM_BIT[5] = {
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
static const double RQ[ADOF] = {
    // カウント方向込
    -2 * PI / res(ARM_BIT[0]),
    -2 * PI / res(ARM_BIT[1]),
    -2 * PI / res(ARM_BIT[2]),
    -2 * PI / res(ARM_BIT[3]),
    2 * PI / res(ARM_BIT[4]),
};
//「0〜max」から「-max/2〜max/2」に変更
int shift_range(unsigned int val, unsigned int range)
{
  if (val < range / 2) //+30000)
    return (int)val;
  else
    return (int)val - (int)range;
}

/**
 * @brief エンコーダオフセット設定の読み込み
 * 
 * @param offset エンコーダオフセット格納先
 */
void ReadEncoderOffset(Ktl::Vector<ADOF> &offset){
  std::ifstream ifs(ENC_OFFSET_FILE);
  //double DEG = 10;
  for( int i = 0; i < ADOF; i++ )
    ifs >> offset[i];

  ifs.close();
  offset.print();
  offset *= 1.0 / DEG; //deg -> rad
}

/*******************************************************************
 *     init_module
 ****************************************************************** */
int init_module(void)
{
    ts01.open("192.168.1.100");

    //--- SSI -----------------------------------------------
    short ssi_clock = 16;      // 16 * 100 ns
    short ssi_timeout = 40000; // / 8; // 40000ns / 8ns  //30usでは短すぎる
    for (int j = 0; j < ADOF; j++)
    {
        ts01.setup_ssi(j, ssi_clock, ARM_BIT[j] + 1, ssi_timeout);
    }
    //各 dizital out channel に pulse 生成準備-----------------------------------------
    //true:パルス入力　false:デジタル入力（非パルス）
    /*ts01.set_dout_mode(0, true);  //パルス
    ts01.set_dout_mode(1, true);  //パルス
    ts01.set_dout_mode(2, true);  //パルス
    ts01.set_dout_mode(3, false); //デジタル入力（非パルス）
    ts01.set_dout_mode(4, true);  //パルス
    ts01.set_dout_mode(5, false); //デジタル入力（非パルス）
    ts01.set_dout_mode(6, true);  //パルス
    ts01.set_dout_mode(7, false); //デジタル入力（非パルス）*/

    //--- counter ---------------------------
    ts01.set_count(0);
    ts01.start_count();
    
    return 1;
}

/*******************************************************************
 *     cleanup_module
 *******************************************************************/
void cleanup_module(void)
{
  ts01.stop_sampling();
  for (int j = 0; j < TS01_DO_CH_NUM; j++)
    output.dout[j] = false;
  for (int j = 0; j < ADOF; j++)
    output.u[j] = 5.0;
  ts01.write_data(&output);
  ts01.close();
  rt_print("control module has been removed.\n");
}
  
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(1000);
    auto node = rclcpp::Node::make_shared("ts01_sensor");
    rclcpp::Logger node_logger = node->get_logger();

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    //TS01の状態に関するmsg
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_status_;
    publisher_status_ = node->create_publisher<std_msgs::msg::Bool>("ts01_status",qos);
    auto msg_status_ = std::make_shared<std_msgs::msg::Bool>();
    msg_status_->data = false;


    //エンコーダに関するmsg
    std_msgs::msg::Float32MultiArray msg_encoder_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_encoder_;
    publisher_encoder_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("ts01_encoder", qos);
    msg_encoder_.data.resize(ADOF);
    for(int i=0; i<ADOF;i++){
      msg_encoder_.data[i] = 0.0;
    }
    Ktl::Vector<ADOF> qoffset;
    ReadEncoderOffset(qoffset);

    //DIに関するmsg
    std_msgs::msg::Int32MultiArray msg_di_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_di_;
    publisher_di_ = node->create_publisher<std_msgs::msg::Int32MultiArray>("ts01_di", qos);
    msg_di_.data.resize(ADOF);
    for(int i=0; i<10;i++){
      msg_di_.data[i] = 0;
    }

    //AIに関するmsg
    std_msgs::msg::Float32MultiArray msg_ai_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_ai_;
    publisher_ai_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("ts01_ai", qos);
    msg_ai_.data.resize(ADOF);
    for(int i=0; i<5;i++){
      msg_ai_.data[i] = 0.0;
    }

    publisher_status_->publish(*msg_status_);
    RCLCPP_INFO(node_logger, "Waiting for opening TS01");
    init_module();
    //~~~~~~~接続待ち~~~~~~~//
    RCLCPP_INFO(node_logger, "TS01 is opened");
    ts01.start_sampling(1000);
    
    msg_status_->data = true;

    while (rclcpp::ok()) {
        ts01.read_autosampling_data(&input); 

        //エンコーダのカウント
        int enc[5];
        enc[0] = shift_range(input.ssi[0] >> 1, 0x0000fffff); //-2^19~2^19
        enc[1] = shift_range(input.ssi[1] >> 1, 0x00007ffff); //-2^17~2^17
        enc[2] = shift_range(input.ssi[2] >> 1, 0x00007ffff); 
        enc[3] = shift_range(input.ssi[3] >> 1, 0x00003ffff); //
        enc[4] = shift_range(input.ssi[4] >> 1, 0x00003ffff); //
        for(size_t i=0; i<msg_encoder_.data.size(); i++){
          //RCLCPP_INFO(node_logger, "Encoder #%d:enc = %f", i, RQ[i] * enc[i] - qoffset[i]);
          msg_encoder_.data[i] = RQ[i] * enc[i];
        }

        //DI
        for(size_t i=0; i<msg_di_.data.size(); i++){
          //RCLCPP_INFO(node_logger, "DI #%zd = %d",i, input.din[i]);
          msg_di_.data[i] = input.din[i];
        }

        //AI
        for(size_t i=0; i<msg_ai_.data.size(); i++){
          //RCLCPP_INFO(node_logger, "AI #%zd = %f", i, input.v[i]);
          msg_ai_.data[i] = input.v[i];
        }

        //publish
        publisher_status_->publish(*msg_status_);
        publisher_encoder_ -> publish(msg_encoder_);
        publisher_di_ -> publish(msg_di_);
        publisher_ai_ -> publish(msg_ai_);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    cleanup_module();
    rclcpp::shutdown();
    return 0;
}