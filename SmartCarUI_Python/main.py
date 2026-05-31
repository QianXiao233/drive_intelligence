"""
SmartCarUI Python 版入口
对应 C++ 版 SmartCarUI/main.cpp
"""

import sys

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMessageBox

from mainwindow import MainWindow

APP_STYLESHEET = """
QMessageBox{background:#131E36;color:#FFFFFF;}
QMessageBox QLabel{color:#FFFFFF;font-size:12px;}
QMessageBox QPushButton{background:#2A3B60;color:#fff;border-radius:6px;padding:5px 15px;}
QMessageBox QPushButton:hover{background:#394E7E;}
"""


def main():
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setStyleSheet(APP_STYLESHEET)

    w = MainWindow()
    w.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
