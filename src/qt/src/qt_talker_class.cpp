#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_talker_class.hpp"

MainDialog::MainDialog(QWidget *parent) : QDialog(parent), node_("talker")
{
    setButton = new QPushButton("publish");

    connect(setButton, SIGNAL(clicked()), this, SLOT(publishString()));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(setButton);
    setLayout(layout);

    publisher_ = node_.create_publisher<std_msgs::msg::String>("chatter");
}

void MainDialog::publishString()
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "string";
    publisher_->publish(*msg);
    RCLCPP_INFO(node_.get_logger(), "pub: %s", msg->data.c_str());
}