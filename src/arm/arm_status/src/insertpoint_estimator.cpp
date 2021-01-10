#include <rclcpp/rclcpp.hpp>
#include "../include/arm_status/insertion.hpp"
#include "../option/options_insertpoint.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 初期設定
    float param = 1000.0;
    size_t flag_save = false;
    if (!parse_command_options(argc, argv, &param, &flag_save))
        return 0;

    auto estimator = std::make_shared<Estimation_InsertPoint>();
    estimator->correction.setParameter(param);
    estimator->correction.setSaveFlag(flag_save);

    rclcpp::spin(estimator);
    rclcpp::shutdown();
    return 0;
}