#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "qt_listener_class.hpp"

MainDialog::MainDialog(QWidget *parent) : QDialog(parent)
{
    lineEdit = new QLineEdit;

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(lineEdit);
    setLayout(layout);

    node_ = rclcpp::Node::make_shared("qt_listener");

    //QoSの設定
    size_t depth = rmw_qos_profile_default.depth;
    rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
    rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
    qos.reliability(reliability_policy);
    
    subscription_ = node_->create_subscription<std_msgs::msg::String>("chatter", qos, std::bind(&MainDialog::stringCallback, this, std::placeholders::_1));

    printf("register\n");
}

void MainDialog::stringCallback(const std_msgs::msg::String::SharedPtr string_msg)
{
    QString text = QString::fromStdString(string_msg->data);
    lineEdit->setText(text);
    RCLCPP_INFO(node_->get_logger(), "sub: %s", string_msg->data.c_str());
}