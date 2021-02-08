#include <rclcpp/rclcpp.hpp>
#include "../include/experiment/pullEndoscope.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PullEndoscope>());
    rclcpp::shutdown();
    return 0;
}