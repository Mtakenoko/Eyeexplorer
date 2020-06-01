#include <QVBoxLayout>

#include "ImageWidget.hpp"

ImageWidget::ImageWidget(QWidget *parent)
    : QWidget(parent)
{
    label = new QLabel(tr("empty"));
    setButton = new QPushButton(tr("Set"));
    lineEdit = new QLineEdit;
    slider = new QSlider;
    graphics = new QGraphicsView;

    connect(setButton, SIGNAL(clicked()), this, SLOT(setLabelText()));

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(label);
    layout->addWidget(lineEdit);
    layout->addWidget(setButton);
    layout->addWidget(slider);
    layout->addWidget(graphics);
    setLayout(layout);
}

void ImageWidget::setLabelText()
{
    QString text = lineEdit->text();
    label->setText(text);
}

void ImageWidget::pushButton_clicked()
{
    cv::Mat cv_image;
    QImage Qt_image(cv_image.data, cv_image.cols, cv_image.rows, QImage::Format_RGB888);
    Qt_image = Qt_image.rgbSwapped();
}

void ImageWidget::renderCVImage(const cv::Mat &image)
{
    if (image.empty())
        return;

    QImage _Qimage(image.data, image.cols, image.rows, QImage::Format_RGB888);
    _qImage = _Qimage.rgbSwapped();
    _image = image;
    QWidget::update();
}

void ImageWidget::paintEvent(QPaintEvent *event)
{
    if (!event->isAccepted())
        return;
    QPainter painter(this);

    if (_qImage.width() == 0 || _qImage.height() == 0)
        return;

    painter.drawImage(QPoint(0, 0), _qImage);
}

void ImageWidget::setImage()
{
    cv::Mat img = cv::imread("sample.png", cv::IMREAD_UNCHANGED);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    _image = img.clone();
}