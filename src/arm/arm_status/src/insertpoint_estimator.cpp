#include <rclcpp/rclcpp.hpp>
#include "../include/arm_status/insertion.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Estimation_InsertPoint>());
    rclcpp::shutdown();
    return 0;
}