#include <rclcpp/rclcpp.hpp>
// #include "../include/map/estimate_eyeball.hpp"
#include "../include/map/estimate_eyeball_insert_point.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Estimation_EyeBall>());
    rclcpp::shutdown();
    return 0;
}