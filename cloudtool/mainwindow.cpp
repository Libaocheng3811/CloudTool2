// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include "mainwindow.h"
#include "ui_MainWindow.h"

#include "edit/boundingbox.h"
#include "edit/color.h"

#include "tool/cutting.h"
#include "tool/pickpoints.h"
#include "tool/filters.h"
#include "tool/rangeimage.h"
#include "tool/keypoints.h"
#include "tool/registration.h"

#include <QDesktopWidget>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QComboBox>

#define  PARENT_ICON_PATH   ":/res/icon/document-open.svg"
#define  CHILD_ICON_PATH    ":/res/icon/view-calendar.svg"

// setupUi()是Ui::MainWindow类的方法，负责初始化界面中的控件。
MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    // resize
    this->setBaseSize(1320, 845);
    QList<QDockWidget*> docks;
    docks.push_back(ui->DataDock);
    docks.push_back(ui->PropertiesDock);
    docks.push_back(ui->ConsoleDock);
    QList<int> size;
    size.push_back(300);
    size.push_back(320);
    size.push_back(140);
    resizeDocks(docks, size, Qt::Orientation::Vertical);

    // connect pointer
    ui->cloudtree->setCloudView(ui->cloudview);
    ui->cloudtree->setConsole(ui->console);
    ui->cloudtree->setPropertiesTable(ui->cloudtable);
    ui->cloudtree->setProgressBar(ui->progress_bar);
    ui->cloudtree->setParentIcon(QIcon(PARENT_ICON_PATH));
    ui->cloudtree->setChildIcon(QIcon(CHILD_ICON_PATH));

    // file
    connect(ui->actionOpen, &QAction::triggered, ui->cloudtree, &ct::CloudTree::addCloud);
    connect(ui->actionSave, &QAction::triggered, ui->cloudtree, &ct::CloudTree::saveSelectedClouds);
    connect(ui->actionClose, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeSelectedClouds);
    connect(ui->actionClose_All, &QAction::triggered, ui->cloudtree, &ct::CloudTree::removeAllClouds);
    connect(ui->actionMerge, &QAction::triggered, ui->cloudtree, &ct::CloudTree::mergeSelectedClouds);
    connect(ui->actionClone, &QAction::triggered, ui->cloudtree, &ct::CloudTree::cloneSelectedClouds);
    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

    // edit
    connect(ui->actionBoundingBox, &QAction::triggered, [=] {this->createLeftDock<BoundingBox>("BoundingBox"); });
    connect(ui->actionColors, &QAction::triggered, [=] {this->createLeftDock<Color>("Color"); });

    // view
    connect(ui->actionResetcamera, &QAction::triggered, ui->cloudview, &ct::CloudView::resetCamera);
    connect(ui->actionTopView, &QAction::triggered, ui->cloudview, &ct::CloudView::setTopView);
    connect(ui->actionFrontView, &QAction::triggered, ui->cloudview, &ct::CloudView::setFrontView);
    connect(ui->actionLeftSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setLeftSideView);
    connect(ui->actionBackView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBackView);
    connect(ui->actionRightSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setRightSideView);
    connect(ui->actionBottomView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBottomView);
    connect(ui->actionShowID, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowId);
    connect(ui->actionShowAxes, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowAxes);
    connect(ui->actionShowFPS, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowFPS);

    // tools
    connect(ui->actionCutting, &QAction::triggered, [=] { this->createDialog<Cutting>("Cutting"); });
    connect(ui->actionPickPoints, &QAction::triggered, [=] {this->createDialog<PickPoints>("PickPoints"); });
    connect(ui->actionFilters, &QAction::triggered, [=] {this->createLeftDock<Filters>("Filters"); });
    connect(ui->actionRangeImage, &QAction::triggered, [=]{this->createDialog<RangeImage>("RangeImage");});
    connect(ui->actionDescriptor, &QAction::triggered, [=]
    {
        this->createLeftDock<Descriptor>("Descriptor");
        if (ct::getDock<Registration>("Registration"))
            ct::getDock<Registration>("Registration")->setDescriptor(ct::getDock<Descriptor>("Descriptor"));
    });
    connect(ui->actionRegistration, &QAction::triggered, [=]
    {
      this->createRightDock<Registration>("Registration");
      if (ct::getDock<Registration>("Registration"))
          ct::getDock<Registration>("Registration")->setDescriptor(ct::getDock<Descriptor>("Descriptor"));
    } );

}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::moveEvent(QMoveEvent *event)
{
    QPoint pos = ui->cloudview->mapToGlobal(QPoint(0, 0));
    emit ui->cloudview->posChanged(pos);
    return QMainWindow::moveEvent(event);
}
