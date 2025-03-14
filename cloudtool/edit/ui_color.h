/********************************************************************************
** Form generated from reading UI file 'color.ui'
**
** Created by: Qt User Interface Compiler version 6.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_COLOR_H
#define UI_COLOR_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Color
{
public:
    QWidget *dockWidgetContents;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_2;
    QComboBox *cbox_type;
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QPushButton *btn_x;
    QPushButton *btn_y;
    QPushButton *btn_z;
    QSpacerItem *verticalSpacer;
    QFrame *line;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *btn_apply;
    QPushButton *btn_reset;

    void setupUi(QDockWidget *Color)
    {
        if (Color->objectName().isEmpty())
            Color->setObjectName("Color");
        Color->resize(241, 218);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName("dockWidgetContents");
        verticalLayout = new QVBoxLayout(dockWidgetContents);
        verticalLayout->setObjectName("verticalLayout");
        groupBox = new QGroupBox(dockWidgetContents);
        groupBox->setObjectName("groupBox");
        verticalLayout_2 = new QVBoxLayout(groupBox);
        verticalLayout_2->setObjectName("verticalLayout_2");
        cbox_type = new QComboBox(groupBox);
        cbox_type->addItem(QString());
        cbox_type->addItem(QString());
        cbox_type->addItem(QString());
        cbox_type->addItem(QString());
        cbox_type->setObjectName("cbox_type");

        verticalLayout_2->addWidget(cbox_type);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(-1, -1, 0, -1);

        verticalLayout_2->addLayout(gridLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        label = new QLabel(groupBox);
        label->setObjectName("label");

        horizontalLayout->addWidget(label);

        btn_x = new QPushButton(groupBox);
        btn_x->setObjectName("btn_x");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Ignored, QSizePolicy::Policy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(btn_x->sizePolicy().hasHeightForWidth());
        btn_x->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(btn_x);

        btn_y = new QPushButton(groupBox);
        btn_y->setObjectName("btn_y");
        sizePolicy.setHeightForWidth(btn_y->sizePolicy().hasHeightForWidth());
        btn_y->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(btn_y);

        btn_z = new QPushButton(groupBox);
        btn_z->setObjectName("btn_z");
        sizePolicy.setHeightForWidth(btn_z->sizePolicy().hasHeightForWidth());
        btn_z->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(btn_z);


        verticalLayout_2->addLayout(horizontalLayout);

        verticalSpacer = new QSpacerItem(0, 0, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        line = new QFrame(groupBox);
        line->setObjectName("line");
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        btn_apply = new QPushButton(groupBox);
        btn_apply->setObjectName("btn_apply");
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/res/icon/apply1.svg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_apply->setIcon(icon);

        horizontalLayout_2->addWidget(btn_apply);

        btn_reset = new QPushButton(groupBox);
        btn_reset->setObjectName("btn_reset");
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/res/icon/reset.svg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_reset->setIcon(icon1);

        horizontalLayout_2->addWidget(btn_reset);


        verticalLayout_2->addLayout(horizontalLayout_2);


        verticalLayout->addWidget(groupBox);

        Color->setWidget(dockWidgetContents);

        retranslateUi(Color);

        cbox_type->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Color);
    } // setupUi

    void retranslateUi(QDockWidget *Color)
    {
        Color->setWindowTitle(QCoreApplication::translate("Color", "Color", nullptr));
        groupBox->setTitle(QString());
        cbox_type->setItemText(0, QCoreApplication::translate("Color", "PointCloud", nullptr));
        cbox_type->setItemText(1, QCoreApplication::translate("Color", "BackGround", nullptr));
        cbox_type->setItemText(2, QCoreApplication::translate("Color", "PointCloud Normals", nullptr));
        cbox_type->setItemText(3, QCoreApplication::translate("Color", "BoundingBox", nullptr));

        cbox_type->setCurrentText(QCoreApplication::translate("Color", "PointCloud", nullptr));
        label->setText(QCoreApplication::translate("Color", "Field Name:", nullptr));
        btn_x->setText(QCoreApplication::translate("Color", "X", nullptr));
        btn_y->setText(QCoreApplication::translate("Color", "Y", nullptr));
        btn_z->setText(QCoreApplication::translate("Color", "Z", nullptr));
        btn_apply->setText(QCoreApplication::translate("Color", "Apply", nullptr));
        btn_reset->setText(QCoreApplication::translate("Color", "Reset", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Color: public Ui_Color {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_COLOR_H
