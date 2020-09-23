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
    const int ts01_status = manager->initialize();
    std::cout << "ts01_status : " << ts01_status << std::endl;
    if (ts01_status == TS01_OPENED)
    {
        RCLCPP_INFO(manager->get_logger(), "TS01 status is opened");
        while (rclcpp::ok())
        {
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
    }

    // TS-01をクローズ
    manager->detatch();
    rclcpp::shutdown();
    return 0;
}