#include <rclcpp/rclcpp.hpp>
#include "../include/map/pullout_endoscope2.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PullOut_Endoscope>());
    rclcpp::shutdown();
    return 0;
}