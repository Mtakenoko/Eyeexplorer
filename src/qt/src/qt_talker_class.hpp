#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>

class MainDialog : public QDialog
{
    Q_OBJECT
public:
    MainDialog(QWidget *parent);
    rclcpp::Node::SharedPtr node_;

private Q_SLOTS:
    void publishString();

private:
    QPushButton *setButton;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};