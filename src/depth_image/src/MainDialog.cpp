#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsPixmapItem>

#include "MainDialog.hpp"

MainDialog::MainDialog(QWidget *parent)
    : QDialog(parent),
      fallguys1("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/fallguys1.png"),
      fallguys2("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/fallguys2.jpg"),
      gui_logo("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/logo.jpg"),
      scale_bar("/home/takeyama/workspace/ros2_eyeexplorer/src/depth_image/data/scale_bar.png"),
      topic_sub_image("endoscope_image"), topic_sub_transform("endoscope_transform"), topic_sub_model("eyeball")
{
  label = new QLabel(tr("Welcoome to DepthCreator"));
  label_camera = new QLabel(tr("Camera Image"));
  label_color = new QLabel(tr("Color Image"));
  label_depth = new QLabel(tr("Depth Image"));
  label_scale_bar = new QLabel(tr("Depth Scale"));
  setButton = new QPushButton(tr("Capture"));
  deleteButton = new QPushButton(tr("Delete"));
  saveButton = new QPushButton(tr("Save"));
  lineEdit = new QLineEdit;
  graphics_camera = new QGraphicsView;
  graphics_color = new QGraphicsView;
  graphics_depth = new QGraphicsView;
  graphics_logo = new QGraphicsView;
  graphics_scale_bar = new QGraphicsView;
  scene_camera = new QGraphicsScene;
  scene_color = new QGraphicsScene;
  scene_depth = new QGraphicsScene;
  scene_logo = new QGraphicsScene;
  scene_scale_bar = new QGraphicsScene;

  // ButtoのSIGNAL SLOT
  connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));
  connect(setButton, SIGNAL(clicked()), this, SLOT(inputDataToggle()));
  connect(deleteButton, SIGNAL(clicked()), this, SLOT(deleteToggle()));
  connect(saveButton, SIGNAL(clicked()), this, SLOT(saveToggle()));

  // 画像情報の入力
  scene_logo->setSceneRect(QRectF(0, 0, 480, 120));
  scene_camera->setSceneRect(QRectF(0, 0, 320, 320));
  scene_color->setSceneRect(QRectF(0, 0, 320, 320));
  scene_depth->setSceneRect(QRectF(0, 0, 320, 320));
  scene_scale_bar->setSceneRect(QRectF(0, 0, 148, 320));

  // 文字のフォント
  QFont font_camera = label_camera->font();
  font_camera.setPointSize(20);
  font_camera.setBold(true);
  label_camera->setFont(font_camera);
  QFont font_color = label_color->font();
  font_color.setPointSize(20);
  font_color.setBold(true);
  label_color->setFont(font_color);
  QFont font_depth = label_depth->font();
  font_depth.setPointSize(20);
  font_depth.setBold(true);
  label_depth->setFont(font_depth);
  QFont font_scale_bar = label_scale_bar->font();
  font_scale_bar.setPointSize(20);
  font_scale_bar.setBold(true);
  label_scale_bar->setFont(font_scale_bar);

  // サイズの固定
  graphics_camera->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  graphics_color->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  graphics_depth->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  graphics_logo->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  graphics_scale_bar->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  // 最初なにもないのもつまらないのでイカちゃん達の画像をいれてる。かわいい。
  MainDialog::initImage(scene_camera, fallguys1);
  MainDialog::initImage(scene_color, fallguys1);
  MainDialog::initImage(scene_depth, fallguys2);
  MainDialog::initImage(scene_logo, gui_logo);
  MainDialog::initImage(scene_scale_bar, scale_bar);
  graphics_camera->setScene(scene_camera);
  graphics_color->setScene(scene_color);
  graphics_depth->setScene(scene_depth);
  graphics_logo->setScene(scene_logo);
  graphics_scale_bar->setScene(scene_scale_bar);

  // 画像とラベルのウィジェット配置
  QVBoxLayout *layout_V_camera = new QVBoxLayout;
  layout_V_camera->addWidget(graphics_camera);
  layout_V_camera->addWidget(label_camera);
  QVBoxLayout *layout_V_color = new QVBoxLayout;
  layout_V_color->addWidget(graphics_color);
  layout_V_color->addWidget(label_color);
  QVBoxLayout *layout_V_depth = new QVBoxLayout;
  layout_V_depth->addWidget(graphics_depth);
  layout_V_depth->addWidget(label_depth);
  QVBoxLayout *layout_V_scale_bar = new QVBoxLayout;
  layout_V_scale_bar->addWidget(graphics_scale_bar);
  layout_V_scale_bar->addWidget(label_scale_bar);

  // 画像のウィジェット配置
  QHBoxLayout *layout_image_H = new QHBoxLayout;
  layout_image_H->addLayout(layout_V_camera);
  layout_image_H->addLayout(layout_V_color);
  layout_image_H->addLayout(layout_V_depth);
  layout_image_H->addLayout(layout_V_scale_bar);

  // ボタンの配置
  QHBoxLayout *layout_Botton_H = new QHBoxLayout;
  layout_Botton_H->addWidget(setButton);
  layout_Botton_H->addWidget(deleteButton);
  layout_Botton_H->addWidget(saveButton);

  // 全体の配置
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
  sprintf(str, "%d Scenes were saved.", depth_create.getSceneNum());
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
  this->setItemToScene(scene_camera, img);
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
  // 8-bits unsigned, NO. OF CHANNELS = 1
  if (img.type() == CV_8UC1)
  {

    QImage _qImage(img.cols, img.rows, QImage::Format_Indexed8);
    QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
    qscene->addItem(image_item);
    return;
  }
  // 8-bits unsigned, NO. OF CHANNELS = 3
  else if (img.type() == CV_8UC3)
  {
    // Copy input Mat
    const uchar *pSrc = (const uchar *)img.data;
    // Create QImage with same dimensions as input Mat
    QImage _qImage(pSrc, img.cols, img.rows, img.step, QImage::Format_RGB888);
    QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
    qscene->addItem(image_item);
    return;
  }
  else if (img.type() == CV_8UC4)
  {
    // Copy input Mat
    const uchar *pSrc = (const uchar *)img.data;
    // Create QImage with same dimensions as input Mat
    QImage _qImage(pSrc, img.cols, img.rows, img.step, QImage::Format_ARGB32);
    QGraphicsPixmapItem *image_item = new QGraphicsPixmapItem(QPixmap::fromImage(_qImage));
    qscene->addItem(image_item);
    return;
  }
  else
  {
    std::cout << "ERROR: Mat could not be converted to QImage." << std::endl;
  }
}