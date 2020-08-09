#include <rclcpp/time_source.hpp>
#include <rclcpp/clock.hpp>
#include "../include/ts01/manage.hpp"

int main(int argc, char *argv[])
{
    // ROS2システムの設定
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(1000);
    auto manager = std::make_shared<Manager>();

    //時間管理
    rclcpp::TimeSource ts(manager);
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    ts.attachClock(clock);

    manager->initialize();

    while (rclcpp::ok())
    {
        manager->setMessage();
        manager->publish();
        rclcpp::spin_some(manager);
        loop_rate.sleep();
    }
    
    manager->detatch();
    sleep(1);
    rclcpp::shutdown();
    return 0;
}