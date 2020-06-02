#ifndef MAIN_DIALOG_HPP_
#define MAIN_DIALOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGraphicsView>

#include "../include/calibrate_arm.hpp"

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
    void calibrateToggle();
    void updateImage();
    void updateMarkerImage();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *setButton;
    QPushButton *calibrateButton;
    QGraphicsView *graphics;
    QGraphicsView *graphics2;
    QGraphicsView *graphics_logo;
    QGraphicsScene *scene;
    QGraphicsScene *scene2;
    QGraphicsScene *scene_logo;
    std::string spla_girl;
    std::string gui_logo;

    void setItemToScene(QGraphicsScene *qscene, const cv::Mat image);
    void initImage(QGraphicsScene *qscene, std::string file_path);

public:
    rclcpp::Node::SharedPtr node_;

private:
    void createROS2node(const char *node_name);
    std::string topic_sub_image;
    std::string topic_sub_joint;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_;

public:
    Calib_Param calib_param;
};
#endif