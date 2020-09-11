#include "../include/ts01/manage.hpp"

int main(int argc, char *argv[])
{
    // ROS2システムの設定
    rclcpp::init(argc, argv);
    rclcpp::WallRate loop_rate(LOOP_RATE);
    std::cout << "Start connect TS-01" << std::endl;

    // TS-01との接続など
    auto manager = std::make_shared<Manager>();
    std::cout << "Initializing..." << std::endl;
    manager->initialize();
    std::cout << "Finisuhed Initializing" << std::endl;

    while (rclcpp::ok())
    {
        // TS-01の入力を読む
        manager->readData();
        
        // Publish用messageに格納
        manager->setMessage();

        // Publish
        manager->publish();

        // TS-01の出力を行う
        manager->outputData();

        rclcpp::spin_some(manager);
        loop_rate.sleep();
    }

    // TS-01シャットダウン
    manager->detatch();
    sleep(1);
    rclcpp::shutdown();
    return 0;
}