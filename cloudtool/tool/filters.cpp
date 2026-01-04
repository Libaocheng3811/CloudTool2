//
// Created by LBC on 2024/12/25.
//

// You may need to build the project (run Qt uic code generator) to get "ui_Filters.h" resolved

#include "filters.h"
#include "ui_filters.h"

#define FILTER_TYPE_PassThrough                         (0)
#define FILTER_TYPE_VoxelGrid                           (1)
#define FILTER_TYPE_StatisticalOutlierRemoval           (2)
#define FILTER_TYPE_RadiusOutlierRemoval                (3)
#define FILTER_TYPE_ConditionalRemoval                  (4)
#define FILTER_TYPE_GridMinimum                         (5)
#define FILTER_TYPE_LocalMaximum                        (6)
#define FILTER_TYPE_ShadowPoints                        (7)

#define FILTER_PRE_FLAG                     "-filter"
#define FILTER_ADD_FLAG                     "filtered-"

Filters::Filters(QWidget *parent) :
        CustomDock(parent), ui(new Ui::Filters),
        m_thread(this)
{
    ui->setupUi(this);

    // qRegisterMetaType： 这是一个 Qt 的元对象系统功能，它的作用是注册自定义类型，以便能够在信号和槽机制中使用，
    // Qt 的信号和槽机制是基于元对象系统（Meta-Object System）的，它要求传递的类型必须是已经注册的类型
    // 将std::string 类型及其引用类型注册到 Qt 的元对象系统中，允许在信号与槽机制中传递string类型和引用的参数
    qRegisterMetaType<std::string>("std::string &");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<ct::ConditionBase::Ptr>("ConditionBase::Ptr &");
    qRegisterMetaType<ct::ConditionBase::Ptr>("ConditionBase::Ptr");

//    QObject::connect(ui->cbox_type, &QComboBox::currentIndexChanged, ui->stackedWidget, &QStackedWidget::setCurrentIndex);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Filters::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Filters::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Filters::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Filters::reset);

    m_filters = new ct::Filters;
    m_filters->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_filters, &QObject::deleteLater);
    connect(this, &Filters::PassThrough, m_filters, &ct::Filters::PassThrough);
    connect(this, &Filters::VoxelGrid, m_filters, &ct::Filters::VoxelGrid);
    connect(this, &Filters::ApproximateVoxelGrid, m_filters, &ct::Filters::ApproximateVoxelGrid);
    connect(this, &Filters::StatisticalOutlierRemoval, m_filters, &ct::Filters::StatisticalOutlierRemoval);
    connect(this, &Filters::RadiusOutlierRemoval, m_filters, &ct::Filters::RadiusOutlierRemoval);
    connect(this, &Filters::ConditionalRemoval, m_filters, &ct::Filters::ConditionalRemoval);
    connect(this, &Filters::GridMinimum, m_filters, &ct::Filters::GridMinimun);
    connect(this, &Filters::LocalMaximum, m_filters, &ct::Filters::LocalMaximum);
    connect(this, &Filters::ShadowPoints, m_filters, &ct::Filters::ShadowPoints);
    connect(m_filters, &ct::Filters::filterResult, this, &Filters::filterResult);
    m_thread.start();

    // ConditionalRemoval
    connect(ui->btn_add_con, &QPushButton::clicked, [=]
    {
        // 获取当前表格行数
        int row = ui->table_condition->rowCount();
        // 增加表格行
        ui->table_condition->setRowCount(row + 1);

        // 如果当前新增行是表格第一行
        if (row == 0)
        {
            QComboBox* con = new QComboBox;
            con->addItems(QStringList({"And", "Or"}));
            // 将下拉框放置在表格的第一行第一列
            ui->table_condition->setCellWidget(row, 0, con);
        }
        // 如果是表格的第二行
        else if (row == 1)
        {
            // 获取第一行第一列单元格小部件，并将其转为QComboBox类型
            QComboBox* con = (QComboBox*)ui->table_condition->cellWidget(0, 0);

            if (con->currentText() == "And")
            {
                // 移除第一行第一列的下拉框，将第一行，第二行的第一列均设置为不可选项QTableWidgetItem，设置内容与con一致
                ui->table_condition->removeCellWidget(0, 0);
                ui->table_condition->setItem(0, 0, new QTableWidgetItem("And"));
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
            }
            else
            {
                ui->table_condition->removeCellWidget(0, 0);
                ui->table_condition->setItem(0, 0, new QTableWidgetItem("Or"));
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("Or"));
            }
        }
        // 如果新增行是第三行及之后
        else
        {
            // 与上面的行设置相同的属性即可
            if (ui->table_condition->item(0, 0)->text() == "And")
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("And"));
            else
                ui->table_condition->setItem(row, 0, new QTableWidgetItem("'Or"));
        }
        // 设置其他列的项
        QComboBox* com = new QComboBox;
        com->addItems(QStringList({"x", "y", "z", "r", "g", "b"}));
        ui->table_condition->setCellWidget(row, 1, com);

        QComboBox* op = new QComboBox;
        op->addItems(QStringList({">", ">=", "<", "<=", "="}));
        ui->table_condition->setCellWidget(row, 2, op);

        QDoubleSpinBox* value = new QDoubleSpinBox;
        value->setDecimals(3);
        value->setRange(-99999, 99999);
        ui->table_condition->setCellWidget(row, 3, value);
    });
    connect(ui->btn_clear_con, &QPushButton::clicked, [=]
    {
        // 获取当前行数，并移除最后一行
        int row = ui->table_condition->rowCount();
        ui->table_condition->removeRow(row - 1);
    });


    // PassThrough
    // 将spinbox中的值传递到滑动条
    connect(ui->dspin_min, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        ui->slider_min->setValue(value * 1000);
    });
    // 它的目的是将 QDoubleSpinBox 的 valueChanged 信号的地址转换为一个特定类型的成员函数指针
    // 因为在连接信号与槽的时候，Qt 的 connect 函数要求信号和槽的类型要一致。因为槽函数是lambda函数，通常不期望槽函数返回任何值，所以它的返回类型是void
    // 如果没有显式指定返回类型，C++11 及以后的标准会自动推导 lambda 表达式的返回类型。由于 lambda 函数中并没有 return 语句，所以返回类型默认为 void。

    /**
     * @brief 总结：信号&QDoubleSpinBox::valueChanged的类型是void(QDoubleSpinBox::*)(double)，类型是一个成员函数指针
     * lambda函数的类型是void，类型是一个普通的函数对象，
     * 那为什么类型不一致却可以这么写呢？---因为Qt会在后台进行适当的转换，它能正确地处理普通函数对象（lambda）和成员函数指针之间的关系，所以即使信号和槽的类型形式不完全一致
     */
    connect(ui->dspin_max, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        ui->slider_max->setValue(value * 1000);
    });
    connect(ui->slider_min, &QSlider::valueChanged, [=](int value)
    {
        ui->dspin_min->setValue((float)value / 1000);
        if (ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->slider_max, &QSlider::valueChanged, [=](int value)
    {
        ui->dspin_max->setValue((float)value / 1000);
        if (ui->check_refresh->isChecked()) this->preview();
    });

    // VoxelGrid
    connect(ui->check_same_value, &QCheckBox::stateChanged, [=](int state)
    {
        // 如果勾选check_same_value复选框，state为true，则禁用dspin_leafy和dspin_leafz；否则启用。
        if (state)
        {
            ui->dspin_leafy->setEnabled(false);
            ui->dspin_leafz->setEnabled(false);
        }
        else
        {
            ui->dspin_leafy->setEnabled(true);
            ui->dspin_leafz->setEnabled(true);
        }
    });

    connect(ui->dspin_leafx, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
    {
        // 如果勾选了相同值复选框，设置y,z的值与x相同
        if (ui->check_same_value->isChecked())
        {
            ui->dspin_leafy->setValue(value);
            ui->dspin_leafz->setValue(value);
        }
        if (ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->dspin_leafy, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->dspin_leafz, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (!ui->check_same_value->isChecked() && ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->check_approximate, &QCheckBox::stateChanged, [=](int state)
    {
        if (state)
            ui->check_reverse->setEnabled(false);
        else
            ui->check_reverse->setEnabled(true);
    });

    // StatisticalOutlierRemoval
    connect(ui->spin_meank, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
    {
        // 同步更新滤波结果
        if (ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->dspin_stddevmulthresh, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });

    // RadiusOutlierRemoval
    connect(ui->dspin_radius, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });
    connect(ui->spin_minneiborsinradius, static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });

    // GridMinimum
    connect(ui->dspin_resolution, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });

    // LocalMaximum
    connect(ui->dspin_radius_3, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });

    // ShadowPoints
    connect(ui->dspin_threshold, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=]
    {
        if (ui->check_refresh->isChecked()) this->preview();
    });

    connect(ui->cbox_field_name, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Filters::getRange);

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->table_condition->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->check_refresh->setChecked(true);
}

Filters::~Filters() {
    // 请求线程m_thread停止运行，
    m_thread.quit();
    //
    if (!m_thread.wait(3000))
    {
        // 如果m_thread线程没有在3s内退出，强制终止该线程，并再次调用wait方法确保主线程会等待m_thread线程彻底终止
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void Filters::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        // 为滤波器设置输入点云
        m_filters->setInputCloud(cloud);
        m_filters->setNegative(ui->check_reverse->isChecked());
        // 判断滤波类型，并发射对应滤波信号
        switch (ui->cbox_type->currentIndex()) {
            case FILTER_TYPE_PassThrough:
                m_cloudview->showInfo("PassThrough", 1);
                emit PassThrough(ui->cbox_field_name->currentText().toStdString(), (float)ui->slider_min->value() / 1000, (float)ui->slider_max->value() / 1000);
                break;
            case FILTER_TYPE_VoxelGrid:
                m_cloudview->showInfo("VoxelGrid", 1);
                emit VoxelGrid(ui->dspin_leafx->value(), ui->dspin_leafy->value(), ui->dspin_leafz->value());
                break;
            case FILTER_TYPE_StatisticalOutlierRemoval:
                m_cloudview->showInfo("StatisticalOutlierRemoval", 1);
                emit StatisticalOutlierRemoval(ui->spin_meank->value(), ui->dspin_stddevmulthresh->value());
                break;
            case FILTER_TYPE_RadiusOutlierRemoval:
                m_cloudview->showInfo("RadiusOutlierRemoval", 1);
                emit RadiusOutlierRemoval(ui->dspin_radius->value(), ui->spin_minneiborsinradius->value());
                break;
            case FILTER_TYPE_ConditionalRemoval:
                m_cloudview->showInfo("ConditionalRemoval", 1);
                emit ConditionalRemoval(this->getCondition());
                break;
            case FILTER_TYPE_GridMinimum:
                m_cloudview->showInfo("GridMinimum", 1);
                emit GridMinimum(ui->dspin_resolution->value());
                break;
            case FILTER_TYPE_LocalMaximum:
                m_cloudview->showInfo("LocalMaximum", 1);
                emit LocalMaximum(ui->dspin_radius_3->value());
                break;
            case FILTER_TYPE_ShadowPoints:
                if (cloud->hasNormals())
                {
                    printW("Please estimate normals first!");
                    return;
                }
                m_cloudview->showInfo("ShadowPoints", 1);
                emit ShadowPoints(ui->dspin_threshold->value());
                break;
        }
        if (!ui->check_refresh->isChecked()) m_cloudtree->showProgressBar();
    }
}

void Filters::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        return;
    }
    for (auto & cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printW(QString("The cloud[id:1%] has no filtered cloud!").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        // 从视图器中移除旧的过滤点云
        m_cloudview->removePointCloud(new_cloud->id());
        // 为过滤后的点云设置新的ID
        new_cloud->setId(FILTER_ADD_FLAG + cloud->id());
        // 将点云添加到文件树中
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_filter_map.erase(cloud->id());
        printI(QString("Add filtered cloud[id:1%] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Filters::apply()
{
    std::vector<ct::Cloud::Ptr > selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto & cloud : selected_clouds)
    {
        if (m_filter_map.find(cloud->id()) == m_filter_map.end())
        {
            printI(QString("The cloud[id:1%] has no filtered cloud!").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_filter_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_filter_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply filtered cloud[id:1%] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Filters::reset()
{
    for (auto &cloud : m_cloudtree->getSelectedClouds())
    {
        m_cloudtree->setCloudChecked(cloud);
    }
    for (auto &cloud : m_filter_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_filter_map.clear();
    m_cloudview->clearInfo();
}

void Filters::filterResult(const ct::Cloud::Ptr &cloud, float time)
{
    // 传入的参数cloud是滤波之后的结果
    printI(QString("Filter cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    QString id = cloud->id();
    cloud->setId(id + FILTER_PRE_FLAG);
    // 在视图器中添加点云、设置点云颜色和大小
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), ct::Color::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_filter_map[id] = cloud;
    if (!ui->check_refresh->isChecked()) m_cloudtree->closeProgressBar();
}

ct::ConditionBase::Ptr Filters::getCondition()
{
    int rowCount = ui->table_condition->rowCount();
    QString condition;
    if (rowCount == 0) return nullptr;
    // 只有一行的情况下，第一行一列单元格控件的类型是QComboBox,而当大于一行的情况下，第一列单元格控件的类型是QWidgetItem
    else if (rowCount == 1)
        condition = ((QComboBox*)ui->table_condition->cellWidget(0, 0))->currentText();
    else
        condition = ui->table_condition->item(0, 0)->text();
    std::string field;
    ct::CompareOp op;
    double value;
    if (condition == "And")
    {
        // ct::ConditionAnd是一个逻辑与条件，创建一个新对象add_cond
        ct::ConditionAnd::Ptr add_cond(new ct::ConditionAnd);
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            // op是从第i行第2列的单元格中的QComboBox获取的当前索引，并转换为ct::CompareOp枚举类型。
            op = ct::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->cellWidget(i, 3))->value();
            // fieldcomp是一个ct::FieldComparison对象，是一个条件比较对象，由字段名、比较操作符和比较值构造而成。
            ct::FieldComparison::Ptr fieldcomp(new ct::FieldComparison(field, op, value));
            // 将条件比较对象添加到逻辑对象中
            add_cond->addComparison(fieldcomp);
        }
        return add_cond;
    }
    else
    {
        ct::ConditionOr::Ptr or_cond(new ct::ConditionOr );
        for (int i = 0; i < rowCount; i++)
        {
            field = ((QComboBox*)ui->table_condition->cellWidget(i, 1))->currentText().toStdString();
            op = ct::CompareOp(((QComboBox*)ui->table_condition->cellWidget(i, 2))->currentIndex());
            value = ((QDoubleSpinBox*)ui->table_condition->cellWidget(i, 3))->value();
            ct::FieldComparison::Ptr fieldcomp(new ct::FieldComparison(field, op, value));
            or_cond->addComparison(fieldcomp);
        }
        return or_cond;
    }
}

void Filters::getRange(int index)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    // 将min初始化为float类型的最大值、将max初始化为负的float类型最大值
    float min = std::numeric_limits<float>::max(), max = -std::numeric_limits<float>::max();
    // 依次获取所选点云所设置字段的范围，并将其设置为滑动条的范围
    for (auto& cloud : selected_clouds)
    {
        switch (index)
        {
            case 0://x
                min = cloud->min().x < min ? cloud->min().x : min;
                max = cloud->max().x > max ? cloud->max().x : max;
                break;
            case 1://y
                min = cloud->min().y < min ? cloud->min().y : min;
                max = cloud->max().y > max ? cloud->max().y : max;
                break;
            case 2://z
                min = cloud->min().z < min ? cloud->min().z : min;
                max = cloud->max().z > max ? cloud->max().z : max;
                break;
            case 3://rgb
                min = 0, max = 1;
                break;
            case 4://curvature
                min = 0, max = 1;
                break;
        }
        min *= 1000, max *= 1000;
        ui->slider_min->setRange(min, max);
        ui->slider_max->setRange(min, max);
        ui->slider_min->setValue(min);
        ui->slider_max->setValue(max);
    }
}
