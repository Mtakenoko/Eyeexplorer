/*****************************************************
 * 
 *   Author : Yura Aoyama
 *   Description : Command TS01 DO output
 *
 ****************************************************/

#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/strategies/allocator_memory_strategy.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unistd.h>
#include <ktl.h>
#include <ktl/rttask.h>
#include <libts01.h>
#include <sched.h>
#include "controller.h"
#include "parameter.h"

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

#include <boost/format.hpp>

TS01 ts01;
TS01OutputData output;

namespace realtime_test
{

class Ts01OutpComponent : public rclcpp::Node
{
public:
  explicit Ts01OutpComponent(const rclcpp::NodeOptions & options) : Node("ts01_outp_component", options)
  {
    ts01.open("192.168.1.100");
    std::cout << "TS01オープン" << std::endl;
    
    // QoS設定
    size_t depth = rmw_qos_profile_default.depth;
    // rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_reliability_policy_t reliability_policy = (rmw_qos_reliability_policy_t)0; // 0->Best effort  1->Reliable
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // chatterトピックのコールバック関数
    auto callback = [this](const std_msgs::msg::Bool::SharedPtr msg) -> void {
      // 受信データをターミナルに出力
      RCLCPP_INFO(this->get_logger(), "%d", msg->data);

      // TS01でDO
      output.dout[0] = (int)msg->data;
      ts01.write_data(&output);
    };

    // chatterトピックの受信設定
    sub_ = create_subscription<std_msgs::msg::Bool>("pulse", callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

}

#include "rclcpp_components/register_node_macro.hpp"

// クラスローダーにコンポーネントを登録
RCLCPP_COMPONENTS_REGISTER_NODE(realtime_test::Ts01OutpComponent)

// int main(int argc, char *argv[])
// {

//   // ========================== こっからExecutor ===============================
//   // Executorを初期化
//   rclcpp::executor::ExecutorArgs args;

//   // Executorのメモリ確保処理をTLSFアルゴリズムで行うTLSFAllocatorに設定．
//   // これにより，ノードのスピン中にメモリの新規割り当てが発生しなくなる．
//   rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
//   args.memory_strategy = memory_strategy;

//   // リアルタイム制御ループでノードのスピン処理を実行する．
//   // 特別なRttExecutorを使用
//   auto executor = std::make_shared<pendulum_control::RttExecutor>(args);

//   // Executorにリアルタイム制御したいノードを追加
//   executor->add_node(node);

//   // 下の方にExecutorの実行もあるよ

//   // ========================== ここまでExecutor ===============================




//   // TS01接続 --------------------------------------------
//   ts01.open("192.168.1.100");
//   std::cout << "TS01オープン" << std::endl;

//   setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<Listener>("pulse");
//   rclcpp::spin(node);
//   rclcpp::shutdown();

//   // TS01終了処理 -------------------------------------
//   ts01.close();

//   return 0;
// }
