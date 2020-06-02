#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QVBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent), topic_sub_image("endoscope_image"), topic_sub_joint("joint_states")
{
  label = new QLabel(tr("empty"));
  setButton = new QPushButton(tr("Set"));
  calibrateButton = new QPushButton(tr("Calibrate"));
  lineEdit = new QLineEdit;
  graphics = new QGraphicsView;
  scene = new QGraphicsScene;

  // ButtoのSIGNAL SLOT
  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(inputDataToggle()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(updateImage()));
  connect(calibrateButton, SIGNAL(clicked()), this, SLOT(calibrateToggle()));

  // 画像情報の入力
  MainDialog::initImage();
  graphics->setScene(scene);

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
  std::cout << "create ROS2 Node" << std::endl;

  // Nodeクラス
  node_ = rclcpp::Node::make_shared(node_name);

  //QoSの設定
  size_t depth = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(history_policy, depth));
  qos.reliability(reliability_policy);

  // Subscribeの設定
  subscription_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_sub_image, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->calib_param.topic_callback_image_(msg);
      });
  subscription_joint_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic_sub_joint, qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->calib_param.topic_callback_joint_(msg);
      });
}

void MainDialog::setLabelText()
{
  QString text = lineEdit->text();
  label->setText(text);
}

void MainDialog::inputDataToggle()
{
  calib_param.setCaptureFlag();
}

void MainDialog::updateImage()
{
  MainDialog::setScene();
  update();
}

void MainDialog::calibrateToggle()
{
  calib_param.setCalibrationFlag();
}
void MainDialog::setScene()
{
  cv::Mat img;
  calib_param.getNewSceneImage(&img);
  if (img.empty())
  {
    std::cout << "img is empty" << std::endl;
    return;
  }
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  scene->addItem(image_item);
}

void MainDialog::initImage()
{
  cv::Mat img;
  img = cv::imread("/home/takeyama/workspace/ros2_eyeexplorer/src/calibration/data/20191020032920502.png", cv::IMREAD_UNCHANGED);
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  scene->addItem(image_item);
}