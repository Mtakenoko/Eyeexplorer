#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDialog>

#include "qt_listener_class.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    MainDialog *mainDialog = new MainDialog(window);
    mainDialog->show();
    // MainDialog::QDialog *dialog = new MainDialog(window);
    // dialog->show();

    rclcpp::WallRate loop_rate(20);
    while (rclcpp::ok())
    {
        // rclcpp::spin_some(std::make_shared<rclcpp::Node>());
        app.processEvents();
        loop_rate.sleep();
    }

    //終了処理
    rclcpp::shutdown();
    delete window;
    delete mainDialog;
    return 0;
}