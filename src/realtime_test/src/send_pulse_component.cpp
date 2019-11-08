/*****************************************************
 *
 *   Author : Yura Aoyama
 *   Description : Send Pulse for TS01 DO output
 *
 ****************************************************/

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

namespace realtime_test
{

class SendPulseComponent : public rclcpp::Node
{
public:
  // マルチOSに対応した共有ライブラリ生成の最適化
  // COMPOSITION_PUBLIC

  // コンストラクタの引数はNodeOptions
  explicit SendPulseComponent(const rclcpp::NodeOptions & options) : Node("send_pulse_component", options)
  {
    // 送信するメッセージ
    msg_ = std::make_shared<std_msgs::msg::Bool>();
    msg_->data = true;

    // QoS設定
    size_t depth = rmw_qos_profile_default.depth;
    // rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_reliability_policy_t reliability_policy = (rmw_qos_reliability_policy_t)0; // 0->Best effort  1->Reliable
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    // タイマ実行されるイベントハンドラ関数
    auto publish_message = [this]() -> void // ラムダ式による関数オブジェクトの定義
    {
      // 送信メッセージをターミナルに表示
      RCLCPP_INFO(this->get_logger(), "%d", msg_->data);

      msg_->data = !msg_->data; // Boolを反転(Pulseを生成)

      // メッセージをパブリッシュ
      pub_->publish(msg_);
    };

    // pulseトピックの送信設定
    pub_ = create_publisher<std_msgs::msg::Bool>("pulse", qos);

    // publish_messageの指定したミリ秒周期でのタイマ実行
    timer_ = create_wall_timer(1ms, publish_message);
  }

private:
  std::shared_ptr<std_msgs::msg::Bool> msg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}


// クラスローダーにコンポーネントを登録
RCLCPP_COMPONENTS_REGISTER_NODE(realtime_test::SendPulseComponent)
