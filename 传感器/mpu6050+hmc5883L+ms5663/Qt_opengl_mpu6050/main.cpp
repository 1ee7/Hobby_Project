#include "mainwindow.h"
//#include "openglwidget.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
//    OpenglWidget w;
    w.show();

    return a.exec();
}
