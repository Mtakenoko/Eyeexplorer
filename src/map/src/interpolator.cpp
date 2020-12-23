#include <rclcpp/rclcpp.hpp>
#include "../include/map/interpolation_eyeball.hpp"
#include "../option/options_interpolation.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Initialize default demo parameters
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    int cut = 10;
    double thresh_distance = 0.003;

    // Configure demo parameters with command line options.
    if (!parse_command_options(argc, argv, &depth, &reliability_policy, &history_policy, &cut, &thresh_distance))
        return 0;

    // Create Gridmap Instance
    auto interpolator = std::make_shared<Interpolation>();
    interpolator->setQoS(depth, reliability_policy, history_policy);
    interpolator->setCutParameter(cut);
    interpolator->setThreshDistance(thresh_distance);

    rclcpp::spin(interpolator);
    rclcpp::shutdown();
    return 0;
}