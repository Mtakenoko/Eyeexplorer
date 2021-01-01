#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent), topic_sub_pointcloud("/occupancy_grid/interpolated_marker")
{
  label = new QLabel(tr("Welcoome to pointcloud_to_pcd node"));
  saveButton = new QPushButton(tr("Save"));
  showButton = new QPushButton(tr("Show"));
  lineEdit = new QLineEdit;

  // ButtoのSIGNAL SLOT
  connect(saveButton, SIGNAL(clicked()), this, SLOT(saveToggle()));
  connect(showButton, SIGNAL(clicked()), this, SLOT(showToggle()));

  // ボタンの配置
  QHBoxLayout *layout_Botton_H = new QHBoxLayout;
  layout_Botton_H->addWidget(saveButton);
  layout_Botton_H->addWidget(showButton);

  // 全体の配置
  QVBoxLayout *layout_V = new QVBoxLayout;
  layout_V->addWidget(label);
  layout_V->addWidget(lineEdit);
  layout_V->addLayout(layout_Botton_H);
  setLayout(layout_V);

  // ROS2 setting
  MainDialog::createROS2node("pointcloud_to_pcd");
}

void MainDialog::createROS2node(const char *node_name)
{
  // Nodeクラス
  node_ = rclcpp::Node::make_shared(node_name);

  //QoSの設定
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  // Subscriberの設定
  subscription_pointcloud_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic_sub_pointcloud, qos,
      [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        this->pointcloud_to_pcd.topic_callback_(msg);
        this->pointcloud_to_pcd.showPointClooud();
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
  sprintf(str, "PointCloud Num : %d.", pointcloud_to_pcd.getPointCloudNum());
  QString qstr(str);
  label->setText(qstr);
}

void MainDialog::showToggle()
{
  pointcloud_to_pcd.showSavedPCD();
}

void MainDialog::saveToggle()
{
  pointcloud_to_pcd.savePCD();
}