#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>

class MainDialog :  public QDialog,
                    public rclcpp::Node
{
    Q_OBJECT
public:
    MainDialog(QWidget *parent);

private:
    QLineEdit *lineEdit;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void stringCallback(const std_msgs::msg::String::SharedPtr msg);
};