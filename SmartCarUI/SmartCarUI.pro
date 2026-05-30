QT       += core gui widgets network

TARGET = SmartCarUI
TEMPLATE = app

CONFIG += c++11

# 使用 pkg-config 自动获取 OpenCV 配置
unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv4
}

SOURCES += main.cpp \
           mainwindow.cpp \
           imagereceiver.cpp \
           behaviordialog.cpp

HEADERS  += mainwindow.h \
            imagereceiver.h \
            behaviordialog.h

FORMS    += mainwindow.ui
