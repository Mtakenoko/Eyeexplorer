#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>

class MainDialog :  public QDialog
{
    Q_OBJECT
public:
    MainDialog(QWidget *parent);

public:
    QLineEdit *lineEdit;
    rclcpp::Node node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void stringCallback(const std_msgs::msg::String::SharedPtr msg);
};