#include <QApplication>

#include "MainDialog.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Widget用
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    MainDialog *dialog = new MainDialog(window);
    dialog->show();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(dialog->node_);
        app.processEvents();
    }

    // 終了処理
    rclcpp::shutdown();
    delete window;
    delete dialog;
    return 0;
}