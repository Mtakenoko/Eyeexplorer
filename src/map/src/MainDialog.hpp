#ifndef MAIN_DIALOG_HPP_
#define MAIN_DIALOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGraphicsView>

#include "../include/map/pointcloud_to_pcd.hpp"

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
    void saveToggle();
    void showToggle();
    void setString();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *saveButton;

public:
    rclcpp::Node::SharedPtr node_;

private:
    void createROS2node(const char *node_name);
    std::string topic_sub_pointcloud;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_pointcloud_;

public:
    PointCloud_to_PCD pointcloud_to_pcd;
};
#endif