#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_talker_class.hpp"

MainDialog::MainDialog(QWidget *parent) : QDialog(parent), node_("qt_talker")
{
    setButton = new QPushButton("publish");

    connect(setButton, SIGNAL(clicked()), this, SLOT(publishString()));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(setButton);
    setLayout(layout);

    //QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);

    publisher_ = node_.create_publisher<std_msgs::msg::String>("chatter", qos);
}

void MainDialog::publishString()
{
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "string";
    publisher_->publish(*msg);
    RCLCPP_INFO(node_.get_logger(), "pub: %s", msg->data.c_str());
}