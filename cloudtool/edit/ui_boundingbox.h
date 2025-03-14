/********************************************************************************
** Form generated from reading UI file 'boundingbox.ui'
**
** Created by: Qt User Interface Compiler version 6.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BOUNDINGBOX_H
#define UI_BOUNDINGBOX_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_BoundingBox
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QCheckBox *check_wireframe;
    QCheckBox *check_points;
    QCheckBox *check_surface;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *rbtn_aabb;
    QRadioButton *rbtn_obb;
    QHBoxLayout *horizontalLayout_3;
    QDoubleSpinBox *dspin_rx;
    QDoubleSpinBox *dspin_ry;
    QDoubleSpinBox *dspin_rz;
    QSpacerItem *verticalSpacer;
    QPushButton *btn_preview;
    QFrame *line;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *btn_apply;
    QPushButton *btn_reset;

    void setupUi(QDockWidget *BoundingBox)
    {
        if (BoundingBox->objectName().isEmpty())
            BoundingBox->setObjectName("BoundingBox");
        BoundingBox->resize(241, 267);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName("dockWidgetContents");
        verticalLayout_2 = new QVBoxLayout(dockWidgetContents);
        verticalLayout_2->setObjectName("verticalLayout_2");
        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName("groupBox");
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setObjectName("verticalLayout");
        label = new QLabel(groupBox);
        label->setObjectName("label");

        verticalLayout->addWidget(label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        check_wireframe = new QCheckBox(groupBox);
        check_wireframe->setObjectName("check_wireframe");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(check_wireframe->sizePolicy().hasHeightForWidth());
        check_wireframe->setSizePolicy(sizePolicy);
        check_wireframe->setChecked(true);

        horizontalLayout->addWidget(check_wireframe);

        check_points = new QCheckBox(groupBox);
        check_points->setObjectName("check_points");
        sizePolicy.setHeightForWidth(check_points->sizePolicy().hasHeightForWidth());
        check_points->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(check_points);

        check_surface = new QCheckBox(groupBox);
        check_surface->setObjectName("check_surface");
        sizePolicy.setHeightForWidth(check_surface->sizePolicy().hasHeightForWidth());
        check_surface->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(check_surface);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        rbtn_aabb = new QRadioButton(groupBox);
        rbtn_aabb->setObjectName("rbtn_aabb");
        sizePolicy.setHeightForWidth(rbtn_aabb->sizePolicy().hasHeightForWidth());
        rbtn_aabb->setSizePolicy(sizePolicy);
        rbtn_aabb->setChecked(true);

        horizontalLayout_2->addWidget(rbtn_aabb);

        rbtn_obb = new QRadioButton(groupBox);
        rbtn_obb->setObjectName("rbtn_obb");
        sizePolicy.setHeightForWidth(rbtn_obb->sizePolicy().hasHeightForWidth());
        rbtn_obb->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(rbtn_obb);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        dspin_rx = new QDoubleSpinBox(groupBox);
        dspin_rx->setObjectName("dspin_rx");
        sizePolicy.setHeightForWidth(dspin_rx->sizePolicy().hasHeightForWidth());
        dspin_rx->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(dspin_rx);

        dspin_ry = new QDoubleSpinBox(groupBox);
        dspin_ry->setObjectName("dspin_ry");
        sizePolicy.setHeightForWidth(dspin_ry->sizePolicy().hasHeightForWidth());
        dspin_ry->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(dspin_ry);

        dspin_rz = new QDoubleSpinBox(groupBox);
        dspin_rz->setObjectName("dspin_rz");
        sizePolicy.setHeightForWidth(dspin_rz->sizePolicy().hasHeightForWidth());
        dspin_rz->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(dspin_rz);


        verticalLayout->addLayout(horizontalLayout_3);

        verticalSpacer = new QSpacerItem(20, 0, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        btn_preview = new QPushButton(groupBox);
        btn_preview->setObjectName("btn_preview");
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/res/icon/preview.svg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_preview->setIcon(icon);

        verticalLayout->addWidget(btn_preview);

        line = new QFrame(groupBox);
        line->setObjectName("line");
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName("horizontalLayout_4");
        btn_apply = new QPushButton(groupBox);
        btn_apply->setObjectName("btn_apply");
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/res/icon/apply1.svg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_apply->setIcon(icon1);

        horizontalLayout_4->addWidget(btn_apply);

        btn_reset = new QPushButton(groupBox);
        btn_reset->setObjectName("btn_reset");
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/res/icon/reset.svg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_reset->setIcon(icon2);

        horizontalLayout_4->addWidget(btn_reset);


        verticalLayout->addLayout(horizontalLayout_4);


        verticalLayout_2->addWidget(groupBox);

        BoundingBox->setWidget(dockWidgetContents);

        retranslateUi(BoundingBox);

        QMetaObject::connectSlotsByName(BoundingBox);
    } // setupUi

    void retranslateUi(QDockWidget *BoundingBox)
    {
        BoundingBox->setWindowTitle(QCoreApplication::translate("BoundingBox", "BoundingBox", nullptr));
        groupBox->setTitle(QString());
        label->setText(QCoreApplication::translate("BoundingBox", "Representation Type:", nullptr));
        check_wireframe->setText(QCoreApplication::translate("BoundingBox", "WireFrame", nullptr));
        check_points->setText(QCoreApplication::translate("BoundingBox", "Point", nullptr));
        check_surface->setText(QCoreApplication::translate("BoundingBox", "Surface", nullptr));
        rbtn_aabb->setText(QCoreApplication::translate("BoundingBox", "AABB(Red)", nullptr));
        rbtn_obb->setText(QCoreApplication::translate("BoundingBox", "OBB(Green)", nullptr));
        dspin_rx->setPrefix(QCoreApplication::translate("BoundingBox", "R:", nullptr));
        dspin_ry->setPrefix(QCoreApplication::translate("BoundingBox", "P:", nullptr));
        dspin_rz->setPrefix(QCoreApplication::translate("BoundingBox", "Y:", nullptr));
        btn_preview->setText(QCoreApplication::translate("BoundingBox", "Preview", nullptr));
        btn_apply->setText(QCoreApplication::translate("BoundingBox", "Apply", nullptr));
        btn_reset->setText(QCoreApplication::translate("BoundingBox", "Reset", nullptr));
    } // retranslateUi

};

namespace Ui {
    class BoundingBox: public Ui_BoundingBox {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOUNDINGBOX_H
