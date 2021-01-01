#ifndef MAIN_DIALOG_HPP_
#define MAIN_DIALOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGraphicsView>

#include "../include/eyemodel/estimate_eyeball.hpp"

class MainDialog : public QDialog
{
    // Q_OBJECTはこのclassがQtのWidgetとして働けるための記述を追加するためのマクロ
    // 必ずclassの宣言の最初に書きます。
    // このマクロを追加することでこのMainDialogクラスがQtのウィジェットとして働ける
    Q_OBJECT
public:
    MainDialog(QWidget *parent);

    // private Q_SLOTS:も同じようにQtのマクロで、SLOTとして使う関数の前に置く
private Q_SLOTS:
    void setLabelText();
    void setString();
    void cancelToggle();
    void setToggle();
    void calcToggle();
    void publish();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *cancelButton;
    QPushButton *setButton;
    QPushButton *calcButton;

public:
    rclcpp::Node::SharedPtr node_;

private:
    void createROS2node(const char *node_name);
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_eyemodel_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_pointcloud_;

public:
    Estimation_EyeBall estimation_EyeBall;
};
#endif