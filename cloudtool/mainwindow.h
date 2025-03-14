//
// Created by LBC on 2024/10/8.
//

#ifndef CLOUDTOOL2_MAINWINDOW_H
#define CLOUDTOOL2_MAINWINDOW_H

#include <QMainWindow>

#include "ui_mainwindow.h"

#include "base/customdialog.h"
#include "base/customdock.h"

QT_BEGIN_NAMESPACE
// 前向声明，它告诉编译器在其他地方（通常是 ui_mainwindow.h 文件）会有一个 Ui::MainWindow 类，
// Ui::MainWindow 继承自 Ui_MainWindow，这意味着 Ui::MainWindow 类是自动生成的 UI 类，它封装了你在 Qt Designer 中设计的界面元素
// Ui::MainWindow 只是一个 UI 布局类，并不代表你自己定义的主窗口类。你的 MainWindow 类会有一个 Ui::MainWindow 类型的成员变量，这个成员变量用于初始化和管理界面控件
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

    template <class T>
    void createLeftDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::LeftDockWidgetArea, ui->PropertiesDock);
    }

    template <class T>
    void createRightDock(const QString& label)
    {
        ct::createDock<T>(this, label, ui->cloudview, ui->cloudtree, ui->console,
                          Qt::RightDockWidgetArea);
    }

    template <class T>
    void createDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console);
    }

protected:
    // 重写基类中的事件处理器函数，当窗口发生移动时，此事件处理器函数被框架自动调用
    void moveEvent(QMoveEvent* event);

private:
    // 定义了一个指向 Ui::MainWindow 的指针（ui），该指针会指向一个 Ui::MainWindow 类型的对象。
    // 通过这个指针，MainWindow 类能够访问 UI 控件，并将它们嵌入到窗口中。
    Ui::MainWindow *ui;
};


#endif //CLOUDTOOL2_MAINWINDOW_H
