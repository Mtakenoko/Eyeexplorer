#include <rclcpp/rclcpp.hpp>
#include "../include/map/pointcloud_filter.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud_Filter>());
    rclcpp::shutdown();
    return 0;
}