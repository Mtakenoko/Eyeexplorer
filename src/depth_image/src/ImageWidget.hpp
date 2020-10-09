#ifndef IMAGE_WIDGET_HPP_
#define IMAGE_WIDGET_HPP_

#ifndef Q_MOC_RUN
#endif

#include <QWidget>
#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QGraphicsView>

#include <QtGui/QImage>
#include <QtGui/QPainter>
#include <QtGui/QPaintEvent>
#include <opencv2/opencv.hpp>

class ImageWidget
    : public QWidget
{
    Q_OBJECT
public:
    ImageWidget(QWidget *parent = 0);

    //private Q_SLOTS:も同じようにQtのマクロで、SLOTとして使う関数の前に置く
private Q_SLOTS:
    void setLabelText();
    void pushButton_clicked();

private:
    QLabel *label;
    QLineEdit *lineEdit;
    QPushButton *setButton;
    QSlider *slider;
    QGraphicsView *graphics;

public:
    void renderCVImage(const cv::Mat &image);
    void setImage();

protected:
    void paintEvent(QPaintEvent *event);

private:
    QImage _qImage;
    cv::Mat _image;
};
#endif