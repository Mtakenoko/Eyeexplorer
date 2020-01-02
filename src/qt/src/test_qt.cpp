#include <QApplication>
#include <QLabel>

int main(int argc, char** argv){
    Qapplication app(argc, argv);
    Qlabel* label = new Qlabel("Hello world!");
    label->show();
    return app.exec();
}