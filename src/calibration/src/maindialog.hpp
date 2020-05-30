#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>

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

public:
    // int getSetiingFlag();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *setButton;

public:
    rclcpp::Node::SharedPtr node_;
    std::string topic_sub_image;
    std::string topic_sub_joint;
};