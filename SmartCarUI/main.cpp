#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    a.setStyleSheet(R"(
    QMessageBox{background:#131E36;color:#FFFFFF;}
    QMessageBox QLabel{color:#FFFFFF;font-size:12px;}
    QMessageBox QPushButton{background:#2A3B60;color:#fff;border-radius:6px;padding:5px 15px;}
    QMessageBox QPushButton:hover{background:#394E7E;}
    )");

    MainWindow w;
    w.show();
    return a.exec();
}