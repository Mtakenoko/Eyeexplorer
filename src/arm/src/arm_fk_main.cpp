#include "rclcpp/rclcpp.hpp"
#include "../include/arm/arm.h"
#include "../include/arm/arm_fk.h"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadEncoder>());
  rclcpp::shutdown();
  return 0;
}