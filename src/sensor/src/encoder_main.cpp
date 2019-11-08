#include <rclcpp/rclcpp.hpp>
#include "../include/sensor/encoder.h"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadEncoder >());
  rclcpp::shutdown();
  return 0;
}