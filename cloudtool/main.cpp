#include "mainwindow.h"
#include <QApplication>
#include <QDesktopWidget>

int main(int argc, char* argv[]) {
    // QApplication接受 argc 和 argv 参数，传递命令行选项给应用程序

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
