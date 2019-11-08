/*****************************************************
 * 
 *   Author : Yura Aoyama
 *   Description : Executor for TS01 DO output & send pulse
 *
 ****************************************************/

#include <execinfo.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <rttest/rttest.h>
#include <tlsf_cpp/tlsf.hpp>

#include "realtime_test/rtt_executor.hpp"

#include "send_pulse_component.cpp"
#include "ts01_outp_component.cpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template <typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rttest_read_args(argc, argv);
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // タイマーコールバック、トピックコールバック等を行うexecutor
  // rclcpp::executors::SingleThreadedExecutor exec;

  // Executorを初期化
  rclcpp::executor::ExecutorArgs args;

  // Executorのメモリ確保処理をTLSFアルゴリズムで行うTLSFAllocatorに設定．
  // これにより，ノードのスピン中にメモリの新規割り当てが発生しなくなる．
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy = std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
  args.memory_strategy = memory_strategy;

  // リアルタイム制御ループでノードのスピン処理を実行する．
  // 特別なRttExecutorを使用
  auto exec = std::make_shared<realtime_test::RttExecutor>(args);

  // Executorにリアルタイム制御したいノードを追加
  // executor->add_node(motor_node);
  // executor->add_node(controller_node);

  // コンポーネントノードのインスタンスを作成しexecutorに登録する
  auto sender = std::make_shared<realtime_test::SendPulseComponent>(options);
  exec->add_node(sender);
  // exec.add_node(sender);

  // コンポーネントノードのインスタンスを作成しexecutorに登録する
  auto outp = std::make_shared<realtime_test::Ts01OutpComponent>(options);
  exec->add_node(outp);
  // exec.add_node(outp);

  // Ctrl-Cが押されるまでexecutorを実行する
  exec->spin();
  // exec.spin();

  rclcpp::shutdown();
  return 0;
}
