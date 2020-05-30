#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>

#include "maindialog.hpp"
#include "../include/calibration/calibrate_arm.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent), topic_sub_image("image"), topic_sub_joint("joint_states")
{
  label = new QLabel(tr("empty"));
  setButton = new QPushButton(tr("Set"));
  lineEdit = new QLineEdit;

  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(label);
  layout->addWidget(lineEdit);
  layout->addWidget(setButton);
  setLayout(layout);

  // ROS2
  node_ = rclcpp::Node::make_shared("arm_param_calibrator");

  //QoSの設定
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  // Subscribeの設定
  auto calib_param = Calib_Param();
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_(node_, topic_sub_image);
  message_filters::Subscriber<sensor_msgs::msg::JointState> sub_arm_(node_, topic_sub_joint);
  message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::JointState> sync_(sub_image_, sub_arm_, 1000);
  sync_.registerCallback(std::bind(&Calib_Param::topic_callback_, calib_param, std::placeholders::_1, std::placeholders::_2));
}

void MainDialog::setLabelText()
{
  QString text = lineEdit->text();
  label->setText(text);
}