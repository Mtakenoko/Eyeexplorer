#include <rclcpp/rclcpp.hpp>
#include "../include/map/occupancy_gridmap.hpp"
#include "../option/options_gridmap.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    double ClampingThresMax = 0.971;
    double ClampingThresMin = 0.1192;
    double OccupancyThres = 0.5;
    double ProbHit = 0.7;
    double ProbMiss = 0.2;
    double Resolution = 0.001;
    double Threshold_display_occ = 0.5;

    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy,
                               &ClampingThresMax, &ClampingThresMin, &OccupancyThres, &ProbHit, &ProbMiss, &Resolution, &Threshold_display_occ))
        return 0;

    // Create Gridmap Instance
    auto gridmap_creator = std::make_shared<Gridmap>();

    // OcTree Settings
    gridmap_creator->set_OcTree_ClampingThresMax(ClampingThresMax);    // default : 0.971
    gridmap_creator->set_OcTree_ClampingThresMin(ClampingThresMin);    // default : 0.1192
    gridmap_creator->set_OcTree_OccupancyThres(OccupancyThres);        // default : 0.5
    gridmap_creator->set_OcTree_ProbHit(ProbHit);                      // default : 0.7
    gridmap_creator->set_OcTree_ProbMiss(ProbMiss);                    // default : 0.4
    gridmap_creator->set_OcTree_Resolution(Resolution);                // default : 0.001
    gridmap_creator->set_threshold_display_occ(Threshold_display_occ); // default : 0.5
    gridmap_creator->print_setting();

    rclcpp::spin(gridmap_creator);
    rclcpp::shutdown();
    return 0;
}