#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    qApp->setStyleSheet(R"(
        QLabel {
            font-family: "微软雅黑";
            font-size: 12px;
        }
        QLineEdit, QTextEdit {
            border: 1px solid #CCCCCC;
            border-radius: 4px;
            padding: 4px;
        }
        QLineEdit:focus, QTextEdit:focus {
            border-color: #2D89EF;
        }
        QMenuBar, QToolBar {
            background-color: #E0E0E0;
        }
        QPushButton {
            background-color: #4CAF50;
            color: white;
            border: none;
            padding: 10px;
        }
        QPushButton:hover {
            background-color: #45A049;
        }
        QPushButton:disabled {
          background-color: #A0A0A0; /* 灰色背景 */
          color: #787878; /* 淡灰色文字，根据需要可以调整 */
        }
    )");
    w.show();
    return a.exec();
}
