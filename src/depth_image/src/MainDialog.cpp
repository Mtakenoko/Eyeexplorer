#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent),
      spla_girl("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/20191020032920502.png"),
      gui_logo("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/eyeexplorer_logo.png"),
      topic_sub_image("endoscope_image"), topic_sub_transform("endoscope_transform"), topic_sub_model("eyeball")
{
  label = new QLabel(tr("empty"));
  setButton = new QPushButton(tr("Capture"));
  deleteButton = new QPushButton(tr("Delete"));
  saveButton = new QPushButton(tr("Save"));
  lineEdit = new QLineEdit;
  graphics = new QGraphicsView;
  graphics_color = new QGraphicsView;
  graphics_depth = new QGraphicsView;
  graphics_logo = new QGraphicsView;
  scene = new QGraphicsScene;
  scene_color = new QGraphicsScene;
  scene_depth = new QGraphicsScene;
  scene_logo = new QGraphicsScene;

  // ButtoのSIGNAL SLOT
  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(inputDataToggle()));
  connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteToggle()));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(saveToggle()));

  // 画像情報の入力
  scene->setSceneRect(QRectF(0, 0, 320, 320));
  scene_color->setSceneRect(QRectF(0, 0, 320, 320));
  scene_depth->setSceneRect(QRectF(0, 0, 320, 320));

  // 最初なにもないのもつまらないのでイカちゃん達の画像をいれてる。かわいい。
  MainDialog::initImage(scene, spla_girl);
  MainDialog::initImage(scene_color, spla_girl);
  MainDialog::initImage(scene_depth, spla_girl);
  MainDialog::initImage(scene_logo, gui_logo);
  graphics->setScene(scene);
  graphics_color->setScene(scene_color);
  graphics_depth->setScene(scene_depth);
  graphics_logo->setScene(scene_logo);

  // GUIのウィジェット配置
  QHBoxLayout *layout_image_H = new QHBoxLayout;
  layout_image_H->addWidget(graphics);
  layout_image_H->addWidget(graphics_color);
  layout_image_H->addWidget(graphics_depth);

  QHBoxLayout *layout_Botton_H = new QHBoxLayout;
  layout_Botton_H->addWidget(setButton);
  layout_Botton_H->addWidget(deleteButton);
  layout_Botton_H->addWidget(saveButton);

  QVBoxLayout *layout_V = new QVBoxLayout;
  layout_V->addWidget(graphics_logo);
  layout_V->addLayout(layout_image_H);
  layout_V->addWidget(label);
  layout_V->addWidget(lineEdit);
  layout_V->addLayout(layout_Botton_H);
  setLayout(layout_V);

  // ROS2 setting
  MainDialog::createROS2node("depth_image_creator");
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
  subscription_image_ = node_->create_subscription<sensor_msgs::msg::Image>(
      topic_sub_image, qos,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_create.topic_callback_image_(msg);
        this->updateNowImage();
        this->updateColorImage();
        this->updateDepthImage();
        this->setString();
      });
  subscription_transform_ = node_->create_subscription<geometry_msgs::msg::Transform>(
      topic_sub_transform, qos,
      [this](const geometry_msgs::msg::Transform::SharedPtr msg) {
        this->depth_create.topic_callback_transform_(msg);
      });
  subscription_model_ = node_->create_subscription<visualization_msgs::msg::Marker>(
      topic_sub_model, qos,
      [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
        this->depth_create.topic_callback_model_(msg);
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
  sprintf(str, "%d Scenes were descrived. ", depth_create.getSceneNum());
  QString qstr(str);
  label->setText(qstr);
}

void MainDialog::inputDataToggle()
{
  depth_create.setCaptureFlag();
}

void MainDialog::deleteToggle()
{
  depth_create.deleteScene();
}

void MainDialog::saveToggle()
{
  depth_create.saveScene();
}

void MainDialog::updateNowImage()
{
  cv::Mat img;
  depth_create.getNowImage(&img);
  if (img.empty())
    return;
  this->setItemToScene(scene, img);
  update();
}

void MainDialog::updateColorImage()
{
  cv::Mat img;
  depth_create.getNewSceneImage(&img);
  if (img.empty())
    return;
  this->setItemToScene(scene_color, img);
  update();
}

void MainDialog::updateDepthImage()
{
  cv::Mat img;
  depth_create.getNewDepthImage(&img);
  if (img.empty())
    return;
  this->setItemToScene(scene_depth, img);
  update();
}

void MainDialog::setItemToScene(QGraphicsScene *qscene, cv::Mat img)
{
  cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
  QImage _qImage(img.data, img.cols, img.rows, QImage::Format_RGB888);
  QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
  qscene->clear();
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