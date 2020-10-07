#ifndef MAIN_DIALOG_HPP_
#define MAIN_DIALOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGraphicsView>

#include "../include/create.hpp"

class MainDialog : public QDialog
{
    //Q_OBJECTはこのclassがQtのWidgetとして働けるための記述を追加するためのマクロ
    //必ずclassの宣言の最初に書きます。
    //このマクロを追加することでこのMainDialogクラスがQtのウィジェットとして働ける
    Q_OBJECT
public:
    MainDialog(QWidget *parent);

    //private Q_SLOTS:も同じようにQtのマクロで、SLOTとして使う関数の前に置く
private Q_SLOTS:
    void setLabelText();
    void inputDataToggle();
    void saveToggle();
    void deleteToggle();
    void updateColorImage();
    void updateDepthImage();
    void setString();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *setButton;
    QPushButton *saveButton;
    QPushButton *deleteButton;
    QGraphicsView *graphics;
    QGraphicsView *graphics_color;
    QGraphicsView *graphics_depth;
    QGraphicsView *graphics_logo;
    QGraphicsScene *scene;
    QGraphicsScene *scene_color;
    QGraphicsScene *scene_depth;
    QGraphicsScene *scene_logo;
    std::string spla_girl;
    std::string gui_logo;

    void setItemToScene(QGraphicsScene *qscene, const cv::Mat image);
    void initImage(QGraphicsScene *qscene, std::string file_path);
    void updateNowImage();

public:
    rclcpp::Node::SharedPtr node_;

private:
    void createROS2node(const char *node_name);
    std::string topic_sub_image;
    std::string topic_sub_transform;
    std::string topic_sub_model;
    std::string topic_pub_color_image;
    std::string topic_pub_depth_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_transform_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_model_;

public:
    Depth_Create depth_create;
};
#endif