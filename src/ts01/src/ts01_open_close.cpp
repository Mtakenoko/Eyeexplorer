#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <ktl.h>
#include <ktl/rttask.h>
#include <libts01.h>

using namespace std::chrono_literals;
TS01 ts01;
#define ADOF 5


/*******************************************************************
 *     init_module
 *******************************************************************/
int init_module(void)
{
    ts01.open("192.168.1.100");

    printf("connected to TS01\n");
    //--- SSI -----------------------------------------------
    short ssi_clock = 16;      // 16 * 100 ns
    short ssi_timeout = 40000; // / 8; // 40000ns / 8ns  //30usでは短すぎる
    static const double ARM_BIT[ADOF] = {
        20, 19, 19, 18, 18};
    for (int j = 0; j < ADOF; j++)
    {
        ts01.setup_ssi(j, ssi_clock, ARM_BIT[j] + 1, ssi_timeout);
    }

    //各 dizital out channel に pulse 生成準備-----------------------------------------
    //true:パルス入力　false:デジタル入力（非パルス）
    ts01.set_dout_mode(0, false);  //デジタル入力（非パルス）
    ts01.set_dout_mode(1, true);  //パルス
    ts01.set_dout_mode(2, true);  //パルス
    ts01.set_dout_mode(3, false); //デジタル入力（非パルス）
    ts01.set_dout_mode(4, true);  //パルス
    ts01.set_dout_mode(5, false); //デジタル入力（非パルス）
    ts01.set_dout_mode(6, true);  //パルス
    ts01.set_dout_mode(7, false); //デジタル入力（非パルス）

    //--- counter ---------------------------
    ts01.set_count(0);
    ts01.start_count();

    printf("Initialized TS01/n");
    return 1;
}

/*******************************************************************
 *     cleanup_module
 *******************************************************************/
void cleanup_module(void)
{
  ts01.stop_sampling();
  TS01OutputData outPut; //TS01の出力
  for (int j = 0; j < TS01_DO_CH_NUM; j++)
    outPut.dout[j] = false;
  for (int j = 0; j < ADOF; j++)
    outPut.u[j] = 5.0;
  ts01.write_data(&outPut);
  ts01.close();
  rt_print("control module has been removed.\n");
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(1000);
    auto node = rclcpp::Node::make_shared("ts01_open_close");
    rclcpp::Logger node_logger = node->get_logger();


    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    // Set quality of service profile based on command line options.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    publisher_ = node->create_publisher<std_msgs::msg::String>("ts01_status",qos);

    TS01InputData input;   //TS01の入力
    TS01OutputData output; //TS01の出力

    ts01.set_dout_mode(0, false);  //デジタル入力（非パルス）
    ts01.open("192.168.1.100");
    RCLCPP_INFO(node_logger, "TS01 is opened");

    while (rclcpp::ok()) {
        /*
        static bool v = true;
        RCLCPP_INFO(node_logger, "Publishing Voltage #%zd", v);
        output.dout[0] = (int)v;
        ts01.write_data(&output);
        v = !v ;
        rclcpp::spin_some(node);
        loop_rate.sleep();*/
    }
    rclcpp::shutdown();
    cleanup_module();
    return 0;
}