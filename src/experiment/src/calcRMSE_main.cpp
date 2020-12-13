#include <rclcpp/rclcpp.hpp>
#include "../include/experiment/calcRMSE.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Estimation_EyeBall>());
    rclcpp::shutdown();
    return 0;
}