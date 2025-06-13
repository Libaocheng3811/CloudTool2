/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QAction>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <base/cloudview.h>
#include "base/cloudtree.h"
#include "base/console.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionSave;
    QAction *actionClose;
    QAction *actionClose_All;
    QAction *actionMerge;
    QAction *actionClone;
    QAction *actionSample;
    QAction *actionQuit;
    QAction *actionColors;
    QAction *actionCoords;
    QAction *actionNormals;
    QAction *actionBoundingBox;
    QAction *actionScale;
    QAction *actionTransformation;
    QAction *actionTopView;
    QAction *actionFrontView;
    QAction *actionLeftSideView;
    QAction *actionBackView;
    QAction *actionRightSideView;
    QAction *actionBottomView;
    QAction *actionResetcamera;
    QAction *actionShowFPS;
    QAction *actionShowAxes;
    QAction *actionShowID;
    QAction *actionTreeSearch;
    QAction *actionFilters;
    QAction *actionKeyPoint;
    QAction *actionSegmentation;
    QAction *actionRegistration;
    QAction *actionDescriptor;
    QAction *actionRecognition;
    QAction *actionPickPoints;
    QAction *actionSurface;
    QAction *actionCutting;
    QAction *actionSampling;
    QAction *actionCorrespondence;
    QAction *actionMeasure;
    QAction *actionSelection;
    QAction *actionConnection;
    QAction *actionPathPlan;
    QAction *actionRangeImage;
    QAction *actionScreenShot;
    QAction *actionTest;
    QAction *actionBoundary;
    QAction *actionPhotoNeo;
    QAction *actionAzureKinect;
    QAction *actionDark;
    QAction *actionLight;
    QAction *actionDArk;
    QAction *actionEnglish;
    QAction *actionChinese;
    QAction *actionShortCutKey;
    QAction *actionAbout;
    QAction *action_Help;
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    ct::CloudView *cloudview;
    QHBoxLayout *horizontalLayout;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuEdit;
    QMenu *menuView;
    QMenu *menuTools;
    QMenu *menuDevices;
    QMenu *menuOptions;
    QMenu *menuTheme;
    QMenu *menuLanguage;
    QMenu *menuHelp;
    QStatusBar *statusbar;
    QDockWidget *DataDock;
    QWidget *DataTree;
    QVBoxLayout *verticalLayout_2;
    ct::CloudTree *cloudtree;
    QProgressBar *progress_bar;
    QDockWidget *PropertiesDock;
    QWidget *PropertiesTable;
    QVBoxLayout *verticalLayout_3;
    QTableWidget *cloudtable;
    QDockWidget *ConsoleDock;
    QWidget *dockWidgetContents_4;
    QVBoxLayout *verticalLayout_4;
    ct::Console *console;
    QToolBar *FileBar;
    QToolBar *EditBar;
    QToolBar *ToolBar;
    QToolBar *ViewBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1300, 845);
        MainWindow->setMinimumSize(QSize(700, 700));
        MainWindow->setBaseSize(QSize(1300, 845));
        MainWindow->setAcceptDrops(true);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName("actionOpen");
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/res/icon/document-new.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionOpen->setIcon(icon);
        actionSave = new QAction(MainWindow);
        actionSave->setObjectName("actionSave");
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/res/icon/document-save.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSave->setIcon(icon1);
        actionClose = new QAction(MainWindow);
        actionClose->setObjectName("actionClose");
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/res/icon/document-close.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionClose->setIcon(icon2);
        actionClose_All = new QAction(MainWindow);
        actionClose_All->setObjectName("actionClose_All");
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/res/icon/edit-clear-history.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionClose_All->setIcon(icon3);
        actionMerge = new QAction(MainWindow);
        actionMerge->setObjectName("actionMerge");
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/res/icon/kr_combine.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionMerge->setIcon(icon4);
        actionClone = new QAction(MainWindow);
        actionClone->setObjectName("actionClone");
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/res/icon/split.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionClone->setIcon(icon5);
        actionSample = new QAction(MainWindow);
        actionSample->setObjectName("actionSample");
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/res/icon/kstars_constellationart.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSample->setIcon(icon6);
        actionQuit = new QAction(MainWindow);
        actionQuit->setObjectName("actionQuit");
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/res/icon/application-exit.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionQuit->setIcon(icon7);
        actionColors = new QAction(MainWindow);
        actionColors->setObjectName("actionColors");
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/res/icon/color-management.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionColors->setIcon(icon8);
        actionCoords = new QAction(MainWindow);
        actionCoords->setObjectName("actionCoords");
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/res/icon/coordinate.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionCoords->setIcon(icon9);
        actionNormals = new QAction(MainWindow);
        actionNormals->setObjectName("actionNormals");
        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/res/icon/normal.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionNormals->setIcon(icon10);
        actionBoundingBox = new QAction(MainWindow);
        actionBoundingBox->setObjectName("actionBoundingBox");
        QIcon icon11;
        icon11.addFile(QString::fromUtf8(":/res/icon/node.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionBoundingBox->setIcon(icon11);
        actionScale = new QAction(MainWindow);
        actionScale->setObjectName("actionScale");
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/res/icon/transform-scale.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionScale->setIcon(icon12);
        actionTransformation = new QAction(MainWindow);
        actionTransformation->setObjectName("actionTransformation");
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/res/icon/exchange-positions.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionTransformation->setIcon(icon13);
        actionTopView = new QAction(MainWindow);
        actionTopView->setObjectName("actionTopView");
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/res/icon/view_top.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionTopView->setIcon(icon14);
        actionFrontView = new QAction(MainWindow);
        actionFrontView->setObjectName("actionFrontView");
        QIcon icon15;
        icon15.addFile(QString::fromUtf8(":/res/icon/view_front.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionFrontView->setIcon(icon15);
        actionLeftSideView = new QAction(MainWindow);
        actionLeftSideView->setObjectName("actionLeftSideView");
        QIcon icon16;
        icon16.addFile(QString::fromUtf8(":/res/icon/view_leftside.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionLeftSideView->setIcon(icon16);
        actionBackView = new QAction(MainWindow);
        actionBackView->setObjectName("actionBackView");
        QIcon icon17;
        icon17.addFile(QString::fromUtf8(":/res/icon/view_back.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionBackView->setIcon(icon17);
        actionRightSideView = new QAction(MainWindow);
        actionRightSideView->setObjectName("actionRightSideView");
        QIcon icon18;
        icon18.addFile(QString::fromUtf8(":/res/icon/view_rightside.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionRightSideView->setIcon(icon18);
        actionBottomView = new QAction(MainWindow);
        actionBottomView->setObjectName("actionBottomView");
        QIcon icon19;
        icon19.addFile(QString::fromUtf8(":/res/icon/view_bottom.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionBottomView->setIcon(icon19);
        actionResetcamera = new QAction(MainWindow);
        actionResetcamera->setObjectName("actionResetcamera");
        QIcon icon20;
        icon20.addFile(QString::fromUtf8(":/res/icon/view-refresh.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionResetcamera->setIcon(icon20);
        actionShowFPS = new QAction(MainWindow);
        actionShowFPS->setObjectName("actionShowFPS");
        actionShowFPS->setCheckable(true);
        actionShowAxes = new QAction(MainWindow);
        actionShowAxes->setObjectName("actionShowAxes");
        actionShowAxes->setCheckable(true);
        actionShowID = new QAction(MainWindow);
        actionShowID->setObjectName("actionShowID");
        actionShowID->setCheckable(true);
        actionTreeSearch = new QAction(MainWindow);
        actionTreeSearch->setObjectName("actionTreeSearch");
        QIcon icon21;
        icon21.addFile(QString::fromUtf8(":/res/icon/distribute-graph-directed.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionTreeSearch->setIcon(icon21);
        actionFilters = new QAction(MainWindow);
        actionFilters->setObjectName("actionFilters");
        QIcon icon22;
        icon22.addFile(QString::fromUtf8(":/res/icon/filter.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionFilters->setIcon(icon22);
        actionKeyPoint = new QAction(MainWindow);
        actionKeyPoint->setObjectName("actionKeyPoint");
        QIcon icon23;
        icon23.addFile(QString::fromUtf8(":/res/icon/snap-bounding-box-center.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionKeyPoint->setIcon(icon23);
        actionSegmentation = new QAction(MainWindow);
        actionSegmentation->setObjectName("actionSegmentation");
        QIcon icon24;
        icon24.addFile(QString::fromUtf8(":/res/icon/view-grid-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSegmentation->setIcon(icon24);
        actionRegistration = new QAction(MainWindow);
        actionRegistration->setObjectName("actionRegistration");
        QIcon icon25;
        icon25.addFile(QString::fromUtf8(":/res/icon/exchange-positions-clockwise.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionRegistration->setIcon(icon25);
        actionDescriptor = new QAction(MainWindow);
        actionDescriptor->setObjectName("actionDescriptor");
        QIcon icon26;
        icon26.addFile(QString::fromUtf8(":/res/icon/feature.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionDescriptor->setIcon(icon26);
        actionRecognition = new QAction(MainWindow);
        actionRecognition->setObjectName("actionRecognition");
        QIcon icon27;
        icon27.addFile(QString::fromUtf8(":/res/icon/recognition.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionRecognition->setIcon(icon27);
        actionPickPoints = new QAction(MainWindow);
        actionPickPoints->setObjectName("actionPickPoints");
        QIcon icon28;
        icon28.addFile(QString::fromUtf8(":/res/icon/edit-node.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionPickPoints->setIcon(icon28);
        actionSurface = new QAction(MainWindow);
        actionSurface->setObjectName("actionSurface");
        QIcon icon29;
        icon29.addFile(QString::fromUtf8(":/res/icon/draw-cuboid.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSurface->setIcon(icon29);
        actionCutting = new QAction(MainWindow);
        actionCutting->setObjectName("actionCutting");
        QIcon icon30;
        icon30.addFile(QString::fromUtf8(":/res/icon/edit-cut.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionCutting->setIcon(icon30);
        actionSampling = new QAction(MainWindow);
        actionSampling->setObjectName("actionSampling");
        QIcon icon31;
        icon31.addFile(QString::fromUtf8(":/res/icon/sampling.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSampling->setIcon(icon31);
        actionCorrespondence = new QAction(MainWindow);
        actionCorrespondence->setObjectName("actionCorrespondence");
        QIcon icon32;
        icon32.addFile(QString::fromUtf8(":/res/icon/draw-connector.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionCorrespondence->setIcon(icon32);
        actionMeasure = new QAction(MainWindow);
        actionMeasure->setObjectName("actionMeasure");
        QIcon icon33;
        icon33.addFile(QString::fromUtf8(":/res/icon/measure.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionMeasure->setIcon(icon33);
        actionSelection = new QAction(MainWindow);
        actionSelection->setObjectName("actionSelection");
        QIcon icon34;
        icon34.addFile(QString::fromUtf8(":/res/icon/node-transform.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionSelection->setIcon(icon34);
        actionConnection = new QAction(MainWindow);
        actionConnection->setObjectName("actionConnection");
        QIcon icon35;
        icon35.addFile(QString::fromUtf8(":/res/icon/kdenlive-select-all.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionConnection->setIcon(icon35);
        actionPathPlan = new QAction(MainWindow);
        actionPathPlan->setObjectName("actionPathPlan");
        QIcon icon36;
        icon36.addFile(QString::fromUtf8(":/res/icon/curve-connector.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionPathPlan->setIcon(icon36);
        actionRangeImage = new QAction(MainWindow);
        actionRangeImage->setObjectName("actionRangeImage");
        QIcon icon37;
        icon37.addFile(QString::fromUtf8(":/res/icon/adjusthsl.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionRangeImage->setIcon(icon37);
        actionScreenShot = new QAction(MainWindow);
        actionScreenShot->setObjectName("actionScreenShot");
        QIcon icon38;
        icon38.addFile(QString::fromUtf8(":/res/icon/screenshoot.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionScreenShot->setIcon(icon38);
        actionTest = new QAction(MainWindow);
        actionTest->setObjectName("actionTest");
        QIcon icon39;
        icon39.addFile(QString::fromUtf8(":/res/icon/run-build.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionTest->setIcon(icon39);
        actionBoundary = new QAction(MainWindow);
        actionBoundary->setObjectName("actionBoundary");
        QIcon icon40;
        icon40.addFile(QString::fromUtf8(":/res/icon/tool_elliptical_selection.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionBoundary->setIcon(icon40);
        actionPhotoNeo = new QAction(MainWindow);
        actionPhotoNeo->setObjectName("actionPhotoNeo");
        QIcon icon41;
        icon41.addFile(QString::fromUtf8(":/res/icon/device.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionPhotoNeo->setIcon(icon41);
        actionAzureKinect = new QAction(MainWindow);
        actionAzureKinect->setObjectName("actionAzureKinect");
        actionAzureKinect->setIcon(icon41);
        actionDark = new QAction(MainWindow);
        actionDark->setObjectName("actionDark");
        actionLight = new QAction(MainWindow);
        actionLight->setObjectName("actionLight");
        actionDArk = new QAction(MainWindow);
        actionDArk->setObjectName("actionDArk");
        actionEnglish = new QAction(MainWindow);
        actionEnglish->setObjectName("actionEnglish");
        actionChinese = new QAction(MainWindow);
        actionChinese->setObjectName("actionChinese");
        actionShortCutKey = new QAction(MainWindow);
        actionShortCutKey->setObjectName("actionShortCutKey");
        QIcon icon42;
        icon42.addFile(QString::fromUtf8(":/res/icon/fcitx-vk-active-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionShortCutKey->setIcon(icon42);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName("actionAbout");
        QIcon icon43;
        icon43.addFile(QString::fromUtf8(":/res/icon/help-info-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        actionAbout->setIcon(icon43);
        action_Help = new QAction(MainWindow);
        action_Help->setObjectName("action_Help");
        QIcon icon44;
        icon44.addFile(QString::fromUtf8(":/res/icon/help-contents-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        action_Help->setIcon(icon44);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName("verticalLayout");
        cloudview = new ct::CloudView(centralwidget);
        cloudview->setObjectName("cloudview");
        horizontalLayout = new QHBoxLayout(cloudview);
        horizontalLayout->setObjectName("horizontalLayout");

        verticalLayout->addWidget(cloudview);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1300, 29));
        QFont font;
        font.setPointSize(10);
        menubar->setFont(font);
        menuFile = new QMenu(menubar);
        menuFile->setObjectName("menuFile");
        menuEdit = new QMenu(menubar);
        menuEdit->setObjectName("menuEdit");
        menuView = new QMenu(menubar);
        menuView->setObjectName("menuView");
        menuTools = new QMenu(menubar);
        menuTools->setObjectName("menuTools");
        menuDevices = new QMenu(menubar);
        menuDevices->setObjectName("menuDevices");
        menuOptions = new QMenu(menubar);
        menuOptions->setObjectName("menuOptions");
        QFont font1;
        font1.setPointSize(9);
        menuOptions->setFont(font1);
        menuTheme = new QMenu(menuOptions);
        menuTheme->setObjectName("menuTheme");
        menuTheme->setFont(font1);
        QIcon icon45;
        icon45.addFile(QString::fromUtf8(":/res/icon/theme.svg"), QSize(), QIcon::Normal, QIcon::Off);
        menuTheme->setIcon(icon45);
        menuLanguage = new QMenu(menuOptions);
        menuLanguage->setObjectName("menuLanguage");
        QIcon icon46;
        icon46.addFile(QString::fromUtf8(":/res/icon/translate.svg"), QSize(), QIcon::Normal, QIcon::Off);
        menuLanguage->setIcon(icon46);
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName("menuHelp");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        DataDock = new QDockWidget(MainWindow);
        DataDock->setObjectName("DataDock");
        DataDock->setMinimumSize(QSize(260, 165));
        DataDock->setFeatures(QDockWidget::DockWidgetClosable|QDockWidget::DockWidgetMovable);
        DataDock->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
        DataTree = new QWidget();
        DataTree->setObjectName("DataTree");
        verticalLayout_2 = new QVBoxLayout(DataTree);
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout_2->setContentsMargins(-1, -1, 9, 3);
        cloudtree = new ct::CloudTree(DataTree);
        QTreeWidgetItem *__qtreewidgetitem = new QTreeWidgetItem();
        __qtreewidgetitem->setTextAlignment(0, Qt::AlignCenter);
        cloudtree->setHeaderItem(__qtreewidgetitem);
        cloudtree->setObjectName("cloudtree");
        cloudtree->setColumnCount(1);
        cloudtree->header()->setVisible(true);

        verticalLayout_2->addWidget(cloudtree);

        progress_bar = new QProgressBar(DataTree);
        progress_bar->setObjectName("progress_bar");
        progress_bar->setMaximumSize(QSize(16777215, 10));
        progress_bar->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 2px solid #aaa;\n"
"    border-radius: 4px;\n"
"    background-color: #f0f0f0;\n"
"    text-align: center;\n"
"    height: 10px;  /* \345\217\257\351\200\211\357\274\232\350\256\276\345\256\232\345\233\272\345\256\232\351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    border-radius: 4px;\n"
"    background-color: qlineargradient(\n"
"        x1: 0, y1: 0, x2: 1, y2: 0,\n"
"        stop: 0 #4caf50,  /* \346\267\261\347\273\277\350\211\262 */\n"
"        stop: 1 #8bc34a   /* \346\265\205\347\273\277\350\211\262 */\n"
"    );\n"
"}"));
        progress_bar->setMaximum(0);
        progress_bar->setValue(-1);
        progress_bar->setTextVisible(false);
        progress_bar->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(progress_bar);

        DataDock->setWidget(DataTree);
        MainWindow->addDockWidget(Qt::LeftDockWidgetArea, DataDock);
        PropertiesDock = new QDockWidget(MainWindow);
        PropertiesDock->setObjectName("PropertiesDock");
        PropertiesDock->setMinimumSize(QSize(260, 149));
        PropertiesDock->setFeatures(QDockWidget::DockWidgetClosable|QDockWidget::DockWidgetMovable);
        PropertiesDock->setAllowedAreas(Qt::AllDockWidgetAreas);
        PropertiesTable = new QWidget();
        PropertiesTable->setObjectName("PropertiesTable");
        verticalLayout_3 = new QVBoxLayout(PropertiesTable);
        verticalLayout_3->setObjectName("verticalLayout_3");
        cloudtable = new QTableWidget(PropertiesTable);
        if (cloudtable->columnCount() < 2)
            cloudtable->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        cloudtable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        cloudtable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        if (cloudtable->rowCount() < 7)
            cloudtable->setRowCount(7);
        QIcon icon47;
        icon47.addFile(QString::fromUtf8(":/res/icon/akonadiconsole.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        __qtablewidgetitem2->setTextAlignment(Qt::AlignLeading|Qt::AlignVCenter);
        __qtablewidgetitem2->setIcon(icon47);
        cloudtable->setItem(0, 0, __qtablewidgetitem2);
        QIcon icon48;
        icon48.addFile(QString::fromUtf8(":/res/icon/view-calendar-week.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        __qtablewidgetitem3->setIcon(icon48);
        cloudtable->setItem(1, 0, __qtablewidgetitem3);
        QIcon icon49;
        icon49.addFile(QString::fromUtf8(":/res/icon/view-group.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        __qtablewidgetitem4->setIcon(icon49);
        cloudtable->setItem(2, 0, __qtablewidgetitem4);
        QIcon icon50;
        icon50.addFile(QString::fromUtf8(":/res/icon/zoom.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        __qtablewidgetitem5->setIcon(icon50);
        cloudtable->setItem(3, 0, __qtablewidgetitem5);
        QIcon icon51;
        icon51.addFile(QString::fromUtf8(":/res/icon/view-app-grid-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        __qtablewidgetitem6->setIcon(icon51);
        cloudtable->setItem(4, 0, __qtablewidgetitem6);
        QIcon icon52;
        icon52.addFile(QString::fromUtf8(":/res/icon/view-reveal-symbolic.svg"), QSize(), QIcon::Normal, QIcon::Off);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        __qtablewidgetitem7->setIcon(icon52);
        cloudtable->setItem(5, 0, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        __qtablewidgetitem8->setIcon(icon10);
        cloudtable->setItem(6, 0, __qtablewidgetitem8);
        cloudtable->setObjectName("cloudtable");
        cloudtable->setStyleSheet(QString::fromUtf8("/* \345\236\202\347\233\264\346\273\232\345\212\250\346\235\241 */\n"
"QScrollBar:vertical {\n"
"    background: transparent;\n"
"    width: 8px;                     /* \350\256\276\347\275\256\346\273\232\345\212\250\346\235\241\345\256\275\345\272\246 */\n"
"    margin: 0px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical {\n"
"    background: #a6a6a6;\n"
"    border-radius: 4px;             /* \345\234\206\350\247\222\346\273\221\345\235\227 */\n"
"    min-height: 20px;\n"
"}\n"
"\n"
"QScrollBar::handle:vertical:hover {\n"
"    background: #888888;\n"
"}\n"
"\n"
"QScrollBar::add-line:vertical,\n"
"QScrollBar::sub-line:vertical {\n"
"    height: 0px;\n"
"}\n"
"\n"
"/* \346\260\264\345\271\263\346\273\232\345\212\250\346\235\241 */\n"
"QScrollBar:horizontal {\n"
"    background: transparent;\n"
"    height: 8px;                    /* \350\256\276\347\275\256\346\273\232\345\212\250\346\235\241\351\253\230\345\272\246 */\n"
"    margin: 0px;\n"
"}\n"
"\n"
"QScrollBar::handle:horizontal {\n"
"    background: #a6a6a6"
                        ";\n"
"    border-radius: 4px;\n"
"    min-width: 20px;\n"
"}\n"
"\n"
"QScrollBar::handle:horizontal:hover {\n"
"    background: #888888;\n"
"}\n"
"\n"
"QScrollBar::add-line:horizontal,\n"
"QScrollBar::sub-line:horizontal {\n"
"    width: 0px;\n"
"}\n"
""));
        cloudtable->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        cloudtable->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        cloudtable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        cloudtable->setShowGrid(false);
        cloudtable->setGridStyle(Qt::CustomDashLine);
        cloudtable->setRowCount(7);
        cloudtable->horizontalHeader()->setVisible(true);
        cloudtable->verticalHeader()->setVisible(false);

        verticalLayout_3->addWidget(cloudtable);

        PropertiesDock->setWidget(PropertiesTable);
        MainWindow->addDockWidget(Qt::LeftDockWidgetArea, PropertiesDock);
        ConsoleDock = new QDockWidget(MainWindow);
        ConsoleDock->setObjectName("ConsoleDock");
        ConsoleDock->setMinimumSize(QSize(134, 149));
        dockWidgetContents_4 = new QWidget();
        dockWidgetContents_4->setObjectName("dockWidgetContents_4");
        verticalLayout_4 = new QVBoxLayout(dockWidgetContents_4);
        verticalLayout_4->setObjectName("verticalLayout_4");
        console = new ct::Console(dockWidgetContents_4);
        console->setObjectName("console");

        verticalLayout_4->addWidget(console);

        ConsoleDock->setWidget(dockWidgetContents_4);
        MainWindow->addDockWidget(Qt::BottomDockWidgetArea, ConsoleDock);
        FileBar = new QToolBar(MainWindow);
        FileBar->setObjectName("FileBar");
        FileBar->setIconSize(QSize(30, 30));
        MainWindow->addToolBar(Qt::TopToolBarArea, FileBar);
        EditBar = new QToolBar(MainWindow);
        EditBar->setObjectName("EditBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, EditBar);
        ToolBar = new QToolBar(MainWindow);
        ToolBar->setObjectName("ToolBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, ToolBar);
        ViewBar = new QToolBar(MainWindow);
        ViewBar->setObjectName("ViewBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, ViewBar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuEdit->menuAction());
        menubar->addAction(menuView->menuAction());
        menubar->addAction(menuTools->menuAction());
        menubar->addAction(menuDevices->menuAction());
        menubar->addAction(menuOptions->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionOpen);
        menuFile->addAction(actionSave);
        menuFile->addAction(actionClose);
        menuFile->addAction(actionClose_All);
        menuFile->addAction(actionMerge);
        menuFile->addAction(actionClone);
        menuFile->addAction(actionSample);
        menuFile->addSeparator();
        menuFile->addAction(actionQuit);
        menuEdit->addAction(actionColors);
        menuEdit->addAction(actionCoords);
        menuEdit->addAction(actionNormals);
        menuEdit->addAction(actionBoundingBox);
        menuEdit->addAction(actionScale);
        menuEdit->addAction(actionTransformation);
        menuView->addAction(actionTopView);
        menuView->addAction(actionFrontView);
        menuView->addAction(actionLeftSideView);
        menuView->addAction(actionBackView);
        menuView->addAction(actionRightSideView);
        menuView->addAction(actionBottomView);
        menuView->addAction(actionResetcamera);
        menuView->addSeparator();
        menuView->addAction(actionShowFPS);
        menuView->addAction(actionShowAxes);
        menuView->addAction(actionShowID);
        menuTools->addAction(actionTreeSearch);
        menuTools->addAction(actionFilters);
        menuTools->addAction(actionKeyPoint);
        menuTools->addAction(actionSegmentation);
        menuTools->addAction(actionRegistration);
        menuTools->addAction(actionDescriptor);
        menuTools->addAction(actionRecognition);
        menuTools->addAction(actionPickPoints);
        menuTools->addAction(actionSurface);
        menuTools->addAction(actionCutting);
        menuTools->addAction(actionSampling);
        menuTools->addAction(actionCorrespondence);
        menuTools->addAction(actionMeasure);
        menuTools->addAction(actionSelection);
        menuTools->addAction(actionConnection);
        menuTools->addAction(actionPathPlan);
        menuTools->addAction(actionRangeImage);
        menuTools->addSeparator();
        menuTools->addAction(actionScreenShot);
        menuTools->addAction(actionTest);
        menuTools->addAction(actionBoundary);
        menuDevices->addAction(actionPhotoNeo);
        menuDevices->addAction(actionAzureKinect);
        menuOptions->addAction(menuTheme->menuAction());
        menuOptions->addAction(menuLanguage->menuAction());
        menuTheme->addAction(actionDark);
        menuTheme->addAction(actionLight);
        menuTheme->addAction(actionDArk);
        menuLanguage->addAction(actionEnglish);
        menuLanguage->addAction(actionChinese);
        menuHelp->addAction(actionShortCutKey);
        menuHelp->addAction(action_Help);
        menuHelp->addAction(actionAbout);
        FileBar->addAction(actionOpen);
        FileBar->addAction(actionClose);
        FileBar->addAction(actionClose_All);
        FileBar->addAction(actionSave);
        FileBar->addAction(actionSample);
        FileBar->addAction(actionMerge);
        FileBar->addAction(actionClone);
        EditBar->addAction(actionColors);
        EditBar->addAction(actionCoords);
        EditBar->addAction(actionNormals);
        EditBar->addAction(actionBoundingBox);
        EditBar->addAction(actionScale);
        EditBar->addAction(actionTransformation);
        ToolBar->addAction(actionKeyPoint);
        ToolBar->addAction(actionDescriptor);
        ToolBar->addAction(actionFilters);
        ToolBar->addAction(actionSampling);
        ToolBar->addAction(actionSegmentation);
        ToolBar->addAction(actionRegistration);
        ToolBar->addAction(actionSurface);
        ToolBar->addAction(actionPickPoints);
        ToolBar->addAction(actionCutting);
        ToolBar->addAction(actionBoundary);
        ToolBar->addAction(actionRangeImage);
        ViewBar->addAction(actionTopView);
        ViewBar->addAction(actionFrontView);
        ViewBar->addAction(actionLeftSideView);
        ViewBar->addAction(actionBackView);
        ViewBar->addAction(actionRightSideView);
        ViewBar->addAction(actionBottomView);
        ViewBar->addAction(actionResetcamera);
        ViewBar->addAction(actionScreenShot);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "\350\276\271\345\235\241\351\232\220\346\202\243\346\231\272\350\203\275\350\257\206\345\210\253\350\275\257\344\273\266", nullptr));
        actionOpen->setText(QCoreApplication::translate("MainWindow", "Open", nullptr));
#if QT_CONFIG(shortcut)
        actionOpen->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+O", nullptr));
#endif // QT_CONFIG(shortcut)
        actionSave->setText(QCoreApplication::translate("MainWindow", "Save", nullptr));
#if QT_CONFIG(shortcut)
        actionSave->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+S", nullptr));
#endif // QT_CONFIG(shortcut)
        actionClose->setText(QCoreApplication::translate("MainWindow", "Close", nullptr));
#if QT_CONFIG(shortcut)
        actionClose->setShortcut(QCoreApplication::translate("MainWindow", "Ctrl+C", nullptr));
#endif // QT_CONFIG(shortcut)
        actionClose_All->setText(QCoreApplication::translate("MainWindow", "Close all", nullptr));
        actionMerge->setText(QCoreApplication::translate("MainWindow", "Merge", nullptr));
        actionClone->setText(QCoreApplication::translate("MainWindow", "Clone", nullptr));
        actionSample->setText(QCoreApplication::translate("MainWindow", "Sample", nullptr));
        actionQuit->setText(QCoreApplication::translate("MainWindow", "Quit", nullptr));
        actionColors->setText(QCoreApplication::translate("MainWindow", "Colors", nullptr));
        actionCoords->setText(QCoreApplication::translate("MainWindow", "Coords", nullptr));
        actionNormals->setText(QCoreApplication::translate("MainWindow", "Normals", nullptr));
        actionBoundingBox->setText(QCoreApplication::translate("MainWindow", "BoundingBos", nullptr));
        actionScale->setText(QCoreApplication::translate("MainWindow", "Scale", nullptr));
        actionTransformation->setText(QCoreApplication::translate("MainWindow", "Transformation", nullptr));
        actionTopView->setText(QCoreApplication::translate("MainWindow", "TopView", nullptr));
        actionFrontView->setText(QCoreApplication::translate("MainWindow", "FrontView", nullptr));
        actionLeftSideView->setText(QCoreApplication::translate("MainWindow", "LeftSideView", nullptr));
        actionBackView->setText(QCoreApplication::translate("MainWindow", "BackView", nullptr));
        actionRightSideView->setText(QCoreApplication::translate("MainWindow", "RightSideView", nullptr));
        actionBottomView->setText(QCoreApplication::translate("MainWindow", "BottomView", nullptr));
        actionResetcamera->setText(QCoreApplication::translate("MainWindow", "Resetcamera", nullptr));
        actionShowFPS->setText(QCoreApplication::translate("MainWindow", "ShowFPS", nullptr));
        actionShowAxes->setText(QCoreApplication::translate("MainWindow", "ShowAxes", nullptr));
        actionShowID->setText(QCoreApplication::translate("MainWindow", "ShowID", nullptr));
        actionTreeSearch->setText(QCoreApplication::translate("MainWindow", "TreeSearch", nullptr));
        actionFilters->setText(QCoreApplication::translate("MainWindow", "Filters", nullptr));
        actionKeyPoint->setText(QCoreApplication::translate("MainWindow", "KeyPoint", nullptr));
        actionSegmentation->setText(QCoreApplication::translate("MainWindow", "Segmentation", nullptr));
        actionRegistration->setText(QCoreApplication::translate("MainWindow", "Registration", nullptr));
        actionDescriptor->setText(QCoreApplication::translate("MainWindow", "Descriptor", nullptr));
        actionRecognition->setText(QCoreApplication::translate("MainWindow", "Recognition", nullptr));
        actionPickPoints->setText(QCoreApplication::translate("MainWindow", "PickPoints", nullptr));
        actionSurface->setText(QCoreApplication::translate("MainWindow", "Surface", nullptr));
        actionCutting->setText(QCoreApplication::translate("MainWindow", "Cutting", nullptr));
        actionSampling->setText(QCoreApplication::translate("MainWindow", "Sampling", nullptr));
        actionCorrespondence->setText(QCoreApplication::translate("MainWindow", "Correspondence", nullptr));
        actionMeasure->setText(QCoreApplication::translate("MainWindow", "Measure", nullptr));
        actionSelection->setText(QCoreApplication::translate("MainWindow", "Selection", nullptr));
        actionConnection->setText(QCoreApplication::translate("MainWindow", "Connection", nullptr));
        actionPathPlan->setText(QCoreApplication::translate("MainWindow", "PathPlan", nullptr));
        actionRangeImage->setText(QCoreApplication::translate("MainWindow", "RangeImage", nullptr));
        actionScreenShot->setText(QCoreApplication::translate("MainWindow", "ScreenShot", nullptr));
        actionTest->setText(QCoreApplication::translate("MainWindow", "Test", nullptr));
        actionBoundary->setText(QCoreApplication::translate("MainWindow", "Boundary", nullptr));
        actionPhotoNeo->setText(QCoreApplication::translate("MainWindow", "PhotoNeo", nullptr));
        actionAzureKinect->setText(QCoreApplication::translate("MainWindow", "AzureKinect", nullptr));
        actionDark->setText(QCoreApplication::translate("MainWindow", "Origin", nullptr));
        actionLight->setText(QCoreApplication::translate("MainWindow", "Light", nullptr));
        actionDArk->setText(QCoreApplication::translate("MainWindow", "Dark", nullptr));
        actionEnglish->setText(QCoreApplication::translate("MainWindow", "English", nullptr));
        actionChinese->setText(QCoreApplication::translate("MainWindow", "Chinese", nullptr));
        actionShortCutKey->setText(QCoreApplication::translate("MainWindow", "ShortCutKey", nullptr));
        actionAbout->setText(QCoreApplication::translate("MainWindow", "About", nullptr));
        action_Help->setText(QCoreApplication::translate("MainWindow", " Help", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuEdit->setTitle(QCoreApplication::translate("MainWindow", "Edit", nullptr));
        menuView->setTitle(QCoreApplication::translate("MainWindow", "View", nullptr));
        menuTools->setTitle(QCoreApplication::translate("MainWindow", "Tools", nullptr));
        menuDevices->setTitle(QCoreApplication::translate("MainWindow", "Devices", nullptr));
        menuOptions->setTitle(QCoreApplication::translate("MainWindow", "Options", nullptr));
        menuTheme->setTitle(QCoreApplication::translate("MainWindow", "Theme", nullptr));
        menuLanguage->setTitle(QCoreApplication::translate("MainWindow", "Language", nullptr));
        menuHelp->setTitle(QCoreApplication::translate("MainWindow", "Help", nullptr));
        DataDock->setWindowTitle(QCoreApplication::translate("MainWindow", "Data Tree", nullptr));
        QTreeWidgetItem *___qtreewidgetitem = cloudtree->headerItem();
        ___qtreewidgetitem->setText(0, QCoreApplication::translate("MainWindow", "PointCloud File", nullptr));
        PropertiesDock->setWindowTitle(QCoreApplication::translate("MainWindow", "Properties", nullptr));
        QTableWidgetItem *___qtablewidgetitem = cloudtable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "Properties", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = cloudtable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "Value", nullptr));

        const bool __sortingEnabled = cloudtable->isSortingEnabled();
        cloudtable->setSortingEnabled(false);
        QTableWidgetItem *___qtablewidgetitem2 = cloudtable->item(0, 0);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "Id", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = cloudtable->item(1, 0);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "Type", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = cloudtable->item(2, 0);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("MainWindow", "Size", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = cloudtable->item(3, 0);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("MainWindow", "Resolution", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = cloudtable->item(4, 0);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("MainWindow", "PointSize", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = cloudtable->item(5, 0);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("MainWindow", "Opacity", nullptr));
        QTableWidgetItem *___qtablewidgetitem8 = cloudtable->item(6, 0);
        ___qtablewidgetitem8->setText(QCoreApplication::translate("MainWindow", "Normals", nullptr));
        cloudtable->setSortingEnabled(__sortingEnabled);

        ConsoleDock->setWindowTitle(QCoreApplication::translate("MainWindow", "Console", nullptr));
        FileBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
        EditBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        ToolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        ViewBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
