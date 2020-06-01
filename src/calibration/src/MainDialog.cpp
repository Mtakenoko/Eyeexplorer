#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QVBoxLayout>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent), topic_sub_image("image"), topic_sub_joint("joint_states")
{
  label = new QLabel(tr("empty"));
  setButton = new QPushButton(tr("Set"));
  calibrateButton = new QPushButton(tr("Calibrate"));
  lineEdit = new QLineEdit;
  graphics = new QGraphicsView;

  // ButtoのSIGNAL SLOT
  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(inputDataToggle()));
  connect(calibrateButton, SIGNAL(clicked()), this, SLOT(calibrateToggle()));

  // 画像情報の入力
  MainDialog::setScene();
  graphics->setScene(&scene);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(graphics);
  layout->addWidget(label);
  layout->addWidget(lineEdit);
  layout->addWidget(setButton);
  layout->addWidget(calibrateButton);
  setLayout(layout);

  // ROS2 setting
  MainDialog::createROS2node("arm_param_calibrator");
}

void MainDialog::createROS2node(const char *node_name)
{
  // ROS2
  node_ = rclcpp::Node::make_shared(node_name);

  //QoSの設定
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  // Subscribeの設定
  // auto calib_param = Calib_Param();
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

void MainDialog::inputDataToggle()
{
  calib_param.flag_set = true;
  printf("counter = %d\n", calib_param.scene_counter);
}

void MainDialog::calibrateToggle()
{
  calib_param.flag_optimize = true;
}

void MainDialog::setScene()
{
  cv::Mat img = cv::imread("/home/takeyama/workspace/ros2_eyeexplorer/src/calibration/data/DSC08982.jpg", cv::IMREAD_UNCHANGED);
  // cv::Mat img = calib_param.getNewSceneImage();
  if (img.empty())
  {
    std::cout << "img is empty" << std::endl;
    return;
  }

  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  _qImage = _qImage.rgbSwapped();
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  scene.addItem(image_item);
}