#include <QApplication>
#include <QVBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>

//ウィジェット内でのイベント（ボタン押したりとか）のことをSIGNALとよぶ
//そのイベントによって駆動されるものをSLOTと呼ぶ

int main(int argc, char** argv)
{
  QApplication app(argc, argv);

  QWidget* window = new QWidget;
  QVBoxLayout* layout  = new QVBoxLayout;
  QPushButton* button  = new QPushButton("Quit");
  QLineEdit*   edit    = new QLineEdit("");
  QLabel*      label   = new QLabel("");

  layout->addWidget(button);
  layout->addWidget(edit);
  layout->addWidget(label);
  window->setLayout(layout);

  // ここではQwidgetが持っているSIGNALを他のQwidgetが持っているSLOTにつなぐ（QObject::connect()）
  // Qwidgetが持っているSIGNALを自作のCallback関数に繋ぐためにはクラス化が必要！
  QObject::connect(button, SIGNAL( clicked() ), &app, SLOT(quit()) );
  QObject::connect(edit, SIGNAL(textChanged(QString)), label, SLOT(setText(QString)) );

  window->show();

  return app.exec();
}