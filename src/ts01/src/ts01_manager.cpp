#include "../include/ts01/manage.hpp"

int main(int argc, char *argv[])
{
    // ROS2システムの設定
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(LOOP_RATE);

    // TS-01との接続など
    auto manager = std::make_shared<Manager>();
    manager->initialize();

    while (rclcpp::ok())
    {
        // TS-01の入力を読む
        manager->readData();
        // Publish用messageに格納
        manager->setMessage();
        // Publish
        manager->publish();

        rclcpp::spin_some(manager);
        loop_rate.sleep();
    }
    
    // TS-01シャットダウン
    manager->detatch();
    sleep(1);
    rclcpp::shutdown();
    return 0;
}