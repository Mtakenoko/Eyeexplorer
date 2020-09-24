#include <rclcpp/rclcpp.hpp>
#include "../include/map/pullout.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto puller = std::make_shared<PullOut_Endoscope>();
    rclcpp::spin(puller);
    rclcpp::shutdown();
    return 0;
}