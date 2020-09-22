/* TS01の管理を行うノード */

#include "../include/ts01/manage.hpp"

int main(int argc, char *argv[])
{
    // ROS2システムの設定
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(LOOP_RATE);

    // TS-01との接続など
    auto manager = std::make_shared<Manager>();
    RCLCPP_INFO(manager->get_logger(), "Connecting...");
    const int opened = manager->initialize();
    while (rclcpp::ok() && opened == 1)
    {
        RCLCPP_INFO(manager->get_logger(), "TS01 status is opened");

        // TS-01の入力を読む
        manager->readData();

        // Publish用messageに格納
        manager->setMessage();

        // Publish
        manager->publish();

        // ノンブロッキングでコールバック関数を読みに行く
        rclcpp::spin_some(manager);

        // TS-01の出力を行う
        manager->outputData();

        loop_rate.sleep();
    }

    // TS-01をクローズ
    manager->detatch();
    rclcpp::shutdown();
    return 0;
}