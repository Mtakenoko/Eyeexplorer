#include <rclcpp/rclcpp.hpp>
#include "../include/map/occupancy_gridmap.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto creator = std::make_shared<Gridmap>();
    creator->set_VoxelMinimumSize(0.0001);
    rclcpp::spin(creator);
    rclcpp::shutdown();
    return 0;
}