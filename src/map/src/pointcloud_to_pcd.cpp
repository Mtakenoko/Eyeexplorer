#include "../include/map/pointcloud_to_pcd.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloud_to_PCD>());
    rclcpp::shutdown();
    return 0;
}