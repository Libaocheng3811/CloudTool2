//
// Created by LBC on 2024/10/8.
//

#ifndef CLOUDTOOL2_MAINWINDOW_H
#define CLOUDTOOL2_MAINWINDOW_H

#include <QMainWindow>

#include "ui_mainwindow.h"

#include "widgets/customdialog.h"
#include "widgets/customdock.h"

QT_BEGIN_NAMESPACE
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

    template <class T>
    void createModalDialog(const QString& label)
    {
        ct::createDialog<T>(this, label, ui->cloudview, ui->cloudtree, ui->console, false, true);
    }

protected:
    void moveEvent(QMoveEvent* event);

private:
    Ui::MainWindow *ui;
};


#endif //CLOUDTOOL2_MAINWINDOW_H
