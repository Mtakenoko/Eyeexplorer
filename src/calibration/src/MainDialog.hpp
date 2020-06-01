#ifndef MAIN_DIALOG_HPP_
#define MAIN_DIALOG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>

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

public:
    // int getSetiingFlag();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *setButton;
    QPushButton *calibrateButton;
    QGraphicsView *graphics;
    QGraphicsScene scene;

    void setScene();

public:
    rclcpp::Node::SharedPtr node_;
    std::string topic_sub_image;
    std::string topic_sub_joint;
    void createROS2node(const char *node_name);

private:
    Calib_Param calib_param;
};
#endif