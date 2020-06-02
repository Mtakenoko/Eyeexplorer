#include <QApplication>
#include <ceres/ceres.h>

#include "MainDialog.hpp"
// #include "ImageWidget.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // // Ceres Solver Logging用　Initializer
    google::InitGoogleLogging(argv[0]);

    // Widget用
    QApplication app(argc, argv);
    QWidget *window = new QWidget;
    MainDialog *dialog = new MainDialog(window);
    dialog->show();
    // ImageWidget widget;
    // widget.show();

    while (rclcpp::ok())
    {
        rclcpp::spin_some(dialog->node_);
        app.processEvents();
    }
    // rclcpp::spin(dialog->node_);

    // 終了処理
    rclcpp::shutdown();
    delete window;
    delete dialog;
    return 0;
}