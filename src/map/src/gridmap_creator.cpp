#include <rclcpp/rclcpp.hpp>
#include "../include/map/occupancy_gridmap.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto creator = std::make_shared<Gridmap>();

    // OcTree Settings
    creator->set_OcTree_ClampingThresMax(0.971);  // default : 0.971
    creator->set_OcTree_ClampingThresMin(0.1192);  // default : 0.1192
    creator->set_OcTree_OccupancyThres(0.5); // default : 0.5
    creator->set_OcTree_ProbHit(0.7);        // default : 0.7
    creator->set_OcTree_ProbMiss(0.2);       // default : 0.4
    creator->set_OcTree_Resolution(0.001);  // default : 0.001

    rclcpp::spin(creator);
    rclcpp::shutdown();
    return 0;
}