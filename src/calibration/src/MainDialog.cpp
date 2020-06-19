#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent),
      spla_girl("/home/takeyama/workspace/ros2_eyeexplorer/src/calibration/data/20191020032920502.png"),
      gui_logo("/home/takeyama/workspace/ros2_eyeexplorer/src/calibration/data/logo2.png"),
      topic_sub_image("endoscope_image"), topic_sub_joint("joint_states")
{
  label = new QLabel(tr("empty"));
  setButton = new QPushButton(tr("Set"));
  calibrateButton = new QPushButton(tr("Calibrate"));
  lineEdit = new QLineEdit;
  graphics = new QGraphicsView;
  graphics_marker = new QGraphicsView;
  graphics_cam = new QGraphicsView;
  graphics_logo = new QGraphicsView;
  scene = new QGraphicsScene;
  scene_marker = new QGraphicsScene;
  scene_cam = new QGraphicsScene;
  scene_logo = new QGraphicsScene;

  // ButtoのSIGNAL SLOT
  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(inputDataToggle()));
  connect(calibrateButton, SIGNAL(clicked()), this, SLOT(calibrateToggle()));

  // 画像情報の入力
  scene->setSceneRect(QRectF(0, 0, 320, 320));
  scene_marker->setSceneRect(QRectF(0, 0, 320, 320));
  scene_cam->setSceneRect(QRectF(0, 0, 320, 320));

  // 最初なにもないのもつまらないのでイカちゃんの画像をいれてる。かわいい。
  MainDialog::initImage(scene, spla_girl);
  MainDialog::initImage(scene_marker, spla_girl);
  MainDialog::initImage(scene_cam, spla_girl);
  MainDialog::initImage(scene_logo, gui_logo);
  graphics->setScene(scene);
  graphics_marker->setScene(scene_marker);
  graphics_cam->setScene(scene_cam);
  graphics_logo->setScene(scene_logo);

  // GUIのウィジェット配置
  QHBoxLayout *layout_image_H = new QHBoxLayout;
  layout_image_H->addWidget(graphics_cam);
  layout_image_H->addWidget(graphics);
  layout_image_H->addWidget(graphics_marker);

  QHBoxLayout *layout_Botton_H = new QHBoxLayout;
  layout_Botton_H->addWidget(setButton);
  layout_Botton_H->addWidget(calibrateButton);

  QVBoxLayout *layout_V = new QVBoxLayout;
  layout_V->addWidget(graphics_logo);
  layout_V->addLayout(layout_image_H);
  layout_V->addWidget(label);
  layout_V->addWidget(lineEdit);
  layout_V->addLayout(layout_Botton_H);
  setLayout(layout_V);

  // ROS2 setting
  MainDialog::createROS2node("arm_param_calibrator");
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

  // Subscribeの設定
  subscription_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_sub_image, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->calib_param.topic_callback_image_(msg);
        this->updateNowImage();
        this->updateSceneImage();
        this->updateMarkerImage();
        this->setString();
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

void MainDialog::setSceneNum()
{
  char str[50];
  sprintf(str, "%d Scenes were descrived. ", calib_param.getSceneNum());
  QString qstr(str);
  label->setText(qstr);
}

void MainDialog::setUsedSceneNum()
{
  char str[100];
  sprintf(str, "Calibrated! (%d marker were used for this calibration) ", calib_param.getUseSceneNum());
  QString qstr(str);
  label->setText(qstr);
}

void MainDialog::setString()
{
  if (calib_param.getFinishFlag())
  {
    // 終わった
    this->setUsedSceneNum();
  }
  else
  {
    // まだ終わってない
    this->setSceneNum();
  }
}

void MainDialog::inputDataToggle()
{
  calib_param.setCaptureFlag();
}

void MainDialog::updateSceneImage()
{
  cv::Mat img;
  calib_param.getNewSceneImage(&img);
  if (img.empty())
    return;
  MainDialog::setItemToScene(scene, img);
  update();
}

void MainDialog::updateMarkerImage()
{
  cv::Mat img;
  calib_param.getNewMarkerImage(&img);
  if (img.empty())
    return;
  MainDialog::setItemToScene(scene_marker, img);
  update();
}

void MainDialog::updateNowImage()
{
  cv::Mat img;
  calib_param.getNowImage(&img);
  if (img.empty())
    return;
  MainDialog::setItemToScene(scene_cam, img);
  update();
}

void MainDialog::calibrateToggle()
{
  calib_param.setCalibrationFlag();
}

void MainDialog::setItemToScene(QGraphicsScene *qscene, const cv::Mat img)
{
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  qscene->addItem(image_item);
}

void MainDialog::initImage(QGraphicsScene *qscene, std::string file_path)
{
  cv::Mat img;
  img = cv::imread(file_path, cv::IMREAD_UNCHANGED);
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  qscene->addItem(image_item);
}