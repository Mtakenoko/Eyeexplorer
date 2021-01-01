#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent) : QDialog(parent)
{
  label = new QLabel(tr("Welcoome to eyemodel_estimator node"));
  setButton = new QPushButton(tr("set"));
  calcButton = new QPushButton(tr("calc"));
  cancelButton = new QPushButton(tr("cancel"));
  lineEdit = new QLineEdit;

  // ButtoのSIGNAL SLOT
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancelToggle()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(setToggle()));
  connect(calcButton, SIGNAL(clicked()), this, SLOT(calcToggle()));

  // ボタンの配置
  QHBoxLayout *layout_Botton_H = new QHBoxLayout;
  layout_Botton_H->addWidget(setButton);
  layout_Botton_H->addWidget(cancelButton);
  layout_Botton_H->addWidget(calcButton);

  // 全体の配置
  QVBoxLayout *layout_V = new QVBoxLayout;
  layout_V->addWidget(label);
  layout_V->addWidget(lineEdit);
  layout_V->addLayout(layout_Botton_H);
  setLayout(layout_V);

  // ROS2 setting
  MainDialog::createROS2node("eyemodel_estimator");
}

void MainDialog::createROS2node(const char *node_name)
{
  // Nodeクラス
  node_ = rclcpp::Node::make_shared(node_name);

  // QoSの設定
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  // Pub/Subの設定
  publisher_eyemodel_ = node_->create_publisher<visualization_msgs::msg::Marker>("/eyemodel", qos);
  publisher_pointcloud_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/pointcloud/arm", qos);
  subscription_ = node_->create_subscription<geometry_msgs::msg::Transform>(
      "/endoscope_transform", qos, [this](const geometry_msgs::msg::Transform::SharedPtr msg) {
        this->estimation_EyeBall.topic_callback_(msg);
        this->publish();
        this->setString();
      });
}

void MainDialog::setLabelText()
{
  QString text = lineEdit->text();
  label->setText(text);
}

void MainDialog::setString()
{
  char str[50];
  sprintf(str, "PointCloud Num : %d.", estimation_EyeBall.getPointCloudNum());
  QString qstr(str);
  label->setText(qstr);
}

void MainDialog::cancelToggle()
{
  this->estimation_EyeBall.cancel();
}
void MainDialog::setToggle()
{
  this->estimation_EyeBall.setInputflag();
}
void MainDialog::calcToggle()
{
  this->estimation_EyeBall.setCalcflag();
}
void MainDialog::publish()
{
}