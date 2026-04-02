// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include "mainwindow.h"
#include "ui_MainWindow.h"

#include "edit/boundingbox.h"
#include "edit/color.h"

#include "tool/cutting.h"
#include "tool/pickpoints.h"
#include "tool/filters.h"
#include "tool/sampling.h"
#include "tool/rangeimage.h"
#include "tool/keypoints.h"
#include "tool/registration.h"

#include "plugins/csfplugin.h"
#include "plugins/vegplugin.h"
#include "plugins/changedetectplugin.h"

#include "python/python_manager.h"
#include "python/python_bridge.h"

#include "python/python_console.h"
#include "python/python_editor.h"

#include <algorithm>

#include <QDesktopWidget>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QFileDialog>
#include <QComboBox>
#include <QHeaderView>

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

    // === Python Console（按需创建，默认不添加到 tab）===
    auto* python_console = new ct::PythonConsole(nullptr);

    // 处理 tab 关闭请求：关闭单个 tab
    connect(ui->consoleTabWidget, &QTabWidget::tabCloseRequested, this, [this, python_console](int index) {
        QString tabText = ui->consoleTabWidget->tabText(index);
        ui->consoleTabWidget->removeTab(index);
        if (tabText == "Python Console") {
            ui->actionPythonConsole->setChecked(false);
        }
    });

    // connect pointer
    ui->cloudtree->setCloudView(ui->cloudview);
    ui->cloudtree->setConsole(ui->console);
    ui->cloudtree->setPropertiesTable(ui->cloudtable);
    ui->cloudtree->setParentIcon(QIcon(PARENT_ICON_PATH));
    ui->cloudtree->setChildIcon(QIcon(CHILD_ICON_PATH));

    // 设置属性表格列宽 - 第一列固定，第二列拉伸
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->cloudtable->setColumnWidth(0, 120);

    // 设置属性表格列宽自适应
    // 第一列（属性名）固定宽度，第二列（值）可拉伸填充
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->cloudtable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->cloudtable->setColumnWidth(0, 140);  // 第一列固定宽度，确保属性名显示完整

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
    connect(ui->actionResetcamera, &QAction::triggered, ui->cloudtree, &ct::CloudTree::zoomToSelected);
    connect(ui->actionTopView, &QAction::triggered, ui->cloudview, &ct::CloudView::setTopView);
    connect(ui->actionFrontView, &QAction::triggered, ui->cloudview, &ct::CloudView::setFrontView);
    connect(ui->actionLeftSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setLeftSideView);
    connect(ui->actionBackView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBackView);
    connect(ui->actionRightSideView, &QAction::triggered, ui->cloudview, &ct::CloudView::setRightSideView);
    connect(ui->actionBottomView, &QAction::triggered, ui->cloudview, &ct::CloudView::setBottomView);
    connect(ui->actionShowID, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowId);
    connect(ui->actionShowAxes, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowAxes);
    connect(ui->actionShowFPS, &QAction::triggered, ui->cloudview, &ct::CloudView::setShowFPS);
    ui->actionShowFPS->setChecked(true);
    ui->actionShowAxes->setChecked(true);
    ui->actionShowID->setChecked(true);
    connect(ui->actionShowDataTree, &QAction::toggled, [=](bool checked){
        ui->DataDock->setVisible(checked); });
    connect(ui->DataDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowDataTree->setChecked(visible);
    });
    connect(ui->actionShowProperties, &QAction::toggled, [=](bool checked){
        ui->PropertiesDock->setVisible(checked); });
    connect(ui->PropertiesDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowProperties->setChecked(visible);
    });
    connect(ui->actionShowConsole, &QAction::toggled, [=](bool checked){
        if (checked) {
            // 确保 Console tab 存在
            int idx = ui->consoleTabWidget->indexOf(ui->console);
            if (idx < 0) {
                ui->consoleTabWidget->insertTab(0, ui->console, "Report Console");
            }
            ui->ConsoleDock->show();
            ui->consoleTabWidget->setCurrentIndex(0);
        } else {
            ui->ConsoleDock->hide();
        }
    });
    connect(ui->ConsoleDock, &QDockWidget::visibilityChanged, [=](bool visible){
        if (!this->isMinimized()) ui->actionShowConsole->setChecked(visible);
    });

    // tools
    connect(ui->actionCutting, &QAction::triggered, [=] { this->createDialog<Cutting>("Cutting"); });
    connect(ui->actionPickPoints, &QAction::triggered, [=] {this->createDialog<PickPoints>("PickPoints"); });
    connect(ui->actionFilters, &QAction::triggered, [=] {this->createLeftDock<Filters>("Filters"); });
    connect(ui->actionSampling, &QAction::triggered, [=] {
        this->createModalDialog<Sampling>("Point Cloud Sampling");
    });
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

    // plugins
    connect(ui->actionCSF, &QAction::triggered, [=] {
        this->createModalDialog<CSFPlugin>("Cloth Simulation Filter");});
    connect(ui->actionVegetation_Filters, &QAction::triggered, [=] {
        this->createModalDialog<VegPlugin>("Vegetation Filters");});
    connect(ui->actionChange_Detection, &QAction::triggered, [=] {
        this->createModalDialog<ChangeDetectPlugin>("Change Detection");});

    // === Python Console（View 菜单，可勾选，默认不打开）===
    connect(ui->actionPythonConsole, &QAction::toggled, this, [this, python_console](bool checked) {
        if (checked) {
            // 添加 Python Console tab（如果尚未添加）
            int idx = ui->consoleTabWidget->indexOf(python_console);
            if (idx < 0) {
                idx = ui->consoleTabWidget->addTab(python_console, "Python Console");
            }
            ui->ConsoleDock->show();
            ui->actionShowConsole->setChecked(true);
            ui->consoleTabWidget->setCurrentIndex(idx);
        } else {
            // 移除 Python Console tab
            int idx = ui->consoleTabWidget->indexOf(python_console);
            if (idx >= 0) {
                ui->consoleTabWidget->removeTab(idx);
            }
        }
    });

    connect(ui->actionPythonEditor, &QAction::toggled, this, [this](bool checked) {
        auto* editor = ui->centralwidget->findChild<ct::PythonEditor*>();
        if (checked) {
            if (!editor) {
                editor = new ct::PythonEditor(ui->centralwidget);
            }
            editor->showEditor();
        } else {
            if (editor) {
                editor->hideEditor();
            }
        }
    });

    connect(ui->actionDark, &QAction::triggered, [=]{
        QFile styleFile(":/res/theme/darkstyle.qss");
        if (styleFile.open(QIODevice::ReadOnly)) {
            QTextStream ts(&styleFile);
            this->setStyleSheet(ts.readAll());
            styleFile.close();
        }
    });

    connect(ui->actionLight, &QAction::triggered, [=]{
        QFile styleFile(":/res/theme/lightstyle.qss");
        if (styleFile.open(QIODevice::ReadOnly)) {
            QTextStream ts(&styleFile);
            this->setStyleSheet(ts.readAll());
            styleFile.close();
        }
    });

    connect(ui->actionOrigin, &QAction::triggered, [=]{
        this->setStyleSheet("");  // 清空样式表，恢复默认
    });

    // === Python Bridge 信号连接 ===
    auto* bridge = ct::PythonManager::instance().bridge();
    if (bridge) {
        // ---- 渲染缓存 ----
        connect(bridge, &ct::PythonBridge::signalCloudChanged,
                ui->cloudview, &ct::CloudView::invalidateCloudRender,
                Qt::QueuedConnection);

        // ---- 注册表同步 ----
        connect(ui->cloudtree, &ct::CloudTree::cloudInserted,
                bridge, &ct::PythonBridge::registerCloud, Qt::QueuedConnection);
        connect(ui->cloudtree, &ct::CloudTree::removedCloudId,
                bridge, &ct::PythonBridge::unregisterCloud, Qt::QueuedConnection);

        // ---- In-use 删除保护 ----
        connect(bridge, &ct::PythonBridge::signalMarkCloudInUse,
                ui->cloudtree, &ct::CloudTree::markCloudInUse, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalUnmarkCloudInUse,
                ui->cloudtree, &ct::CloudTree::unmarkCloudInUse, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalReleaseAllInUse,
                ui->cloudtree, &ct::CloudTree::releaseAllInUse, Qt::QueuedConnection);

        // ---- 日志 → Console ----
        connect(bridge, &ct::PythonBridge::signalLog,
                ui->console, [this](int level, const QString& msg) {
            ui->console->print(static_cast<ct::log_level>(level), msg);
        }, Qt::QueuedConnection);

        // ---- 进度 → CloudTree ----
        connect(bridge, &ct::PythonBridge::signalShowProgress,
                ui->cloudtree, &ct::CloudTree::showProgress, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetProgress,
                ui->cloudtree, &ct::CloudTree::setProgress, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalCloseProgress,
                ui->cloudtree, &ct::CloudTree::closeProgress, Qt::QueuedConnection);

        // ---- 视图控制 → CloudView ----
        connect(bridge, &ct::PythonBridge::signalRefreshView,
                ui->cloudview, [this]() { ui->cloudview->refresh(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalResetCamera,
                ui->cloudview, [this]() { ui->cloudview->resetCamera(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalZoomToBounds,
                ui->cloudtree, &ct::CloudTree::zoomToSelected, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetAutoRender,
                ui->cloudview, [this](bool en) { ui->cloudview->setAutoRender(en); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalZoomToSelected,
                ui->cloudtree, &ct::CloudTree::zoomToSelected, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetTopView,
                ui->cloudview, [this]() { ui->cloudview->setTopView(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetFrontView,
                ui->cloudview, [this]() { ui->cloudview->setFrontView(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetBackView,
                ui->cloudview, [this]() { ui->cloudview->setBackView(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetLeftSideView,
                ui->cloudview, [this]() { ui->cloudview->setLeftSideView(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetRightSideView,
                ui->cloudview, [this]() { ui->cloudview->setRightSideView(); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetBottomView,
                ui->cloudview, [this]() { ui->cloudview->setBottomView(); },
                Qt::QueuedConnection);

        // ---- 点云外观 → CloudView ----
        connect(bridge, &ct::PythonBridge::signalSetPointSize,
                ui->cloudview, [this](const QString& id, float s) {
            ui->cloudview->setPointCloudSize(id, s);
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetOpacity,
                ui->cloudview, [this](const QString& id, float v) {
            ui->cloudview->setPointCloudOpacity(id, v);
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetCloudColorRGB,
                ui->cloudview, [this](const QString& id, float r, float g, float b) {
            ui->cloudview->setPointCloudColor(id, ct::RGB{
                static_cast<uint8_t>(std::min(std::max(r * 255.f, 0.f), 255.f)),
                static_cast<uint8_t>(std::min(std::max(g * 255.f, 0.f), 255.f)),
                static_cast<uint8_t>(std::min(std::max(b * 255.f, 0.f), 255.f))});
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetCloudColorByAxis,
                this, [this, bridge](const QString& id, const QString& axis) {
            auto cloud = bridge->getCloud(id);
            if (cloud) ui->cloudview->setPointCloudColor(cloud, axis);
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalResetCloudColor,
                this, [this, bridge](const QString& id) {
            auto cloud = bridge->getCloud(id);
            if (cloud) ui->cloudview->resetPointCloudColor(cloud);
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalSetCloudVisibility,
                ui->cloudview, [this](const QString& id, bool v) {
            ui->cloudview->setPointCloudVisibility(id, v);
        }, Qt::QueuedConnection);

        // ---- 场景外观 → CloudView ----
        connect(bridge, &ct::PythonBridge::signalSetBackgroundColor,
                ui->cloudview, [this](float r, float g, float b) {
            ui->cloudview->setBackgroundColor(ct::RGB{
                static_cast<uint8_t>(std::min(std::max(r * 255.f, 0.f), 255.f)),
                static_cast<uint8_t>(std::min(std::max(g * 255.f, 0.f), 255.f)),
                static_cast<uint8_t>(std::min(std::max(b * 255.f, 0.f), 255.f))});
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalResetBackgroundColor,
                ui->cloudview, [this]() { ui->cloudview->resetBackgroundColor(); },
                Qt::QueuedConnection);

        // ---- 显示开关 → CloudView ----
        connect(bridge, &ct::PythonBridge::signalShowId,
                ui->cloudview, [this](bool en) { ui->cloudview->setShowId(en); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalShowAxes,
                ui->cloudview, [this](bool en) { ui->cloudview->setShowAxes(en); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalShowFPS,
                ui->cloudview, [this](bool en) { ui->cloudview->setShowFPS(en); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalShowInfo,
                ui->cloudview, [this](const QString& text) {
            ui->cloudview->showInfo(text, 1);
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalClearInfo,
                ui->cloudview, [this]() { ui->cloudview->clearInfo(); },
                Qt::QueuedConnection);

        // ---- 叠加物 → CloudView ----
        connect(bridge, &ct::PythonBridge::signalAddCube,
                ui->cloudview, [this](float cx, float cy, float cz, float size, const QString& id) {
            ct::Box box;
            box.translation = Eigen::Vector3f(cx, cy, cz);
            box.width = box.height = box.depth = size;
            ui->cloudview->addCube(box, id);
        }, Qt::QueuedConnection);
        // TODO: CloudView::add3DLabel 未实现，暂时注释
        // connect(bridge, &ct::PythonBridge::signalAdd3DLabel,
        //         ui->cloudview, [this](const QString& text, float x, float y, float z, const QString& id) {
        //     ct::PointXYZRGBN pos;
        //     pos.x = x; pos.y = y; pos.z = z;
        //     ui->cloudview->add3DLabel(pos, text, id);
        // }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalRemoveShape,
                ui->cloudview, [this](const QString& id) { ui->cloudview->removeShape(id); },
                Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalRemoveAllShapes,
                ui->cloudview, [this]() { ui->cloudview->removeAllShapes(); },
                Qt::QueuedConnection);

        // ---- 点云管理 → CloudTree ----
        connect(bridge, &ct::PythonBridge::signalInsertCloud,
                ui->cloudtree, [this](ct::Cloud::Ptr cloud) {
            ui->cloudtree->insertCloud(cloud);
            ui->cloudview->addPointCloud(cloud);
            ui->cloudview->refresh();
        }, Qt::QueuedConnection);
        connect(bridge, &ct::PythonBridge::signalRemoveSelectedClouds,
                ui->cloudtree, &ct::CloudTree::removeSelectedClouds, Qt::QueuedConnection);
    }

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
