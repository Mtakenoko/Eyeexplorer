/*****************************************************
 * 
 *   Author : Yura Aoyama
 *   Description : Executor for TS01 DO output & send pulse
 *
 ****************************************************/

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "send_pulse_component.cpp"
#include "ts01_outp_component.cpp"

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto talker = std::make_shared<>("chatter");
  auto listener = std::make_shared<Listener>("chatter");
  exec.add_node(talker);
  exec.add_node(listener);
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
