//
// Created by LBC on 2026/3/24.
//

#include "sampling.h"
#include "ui_sampling.h"

Sampling::Sampling(QWidget* parent)
        : CustomDialog(parent), ui(new Ui::Sampling), m_thread(this)
{
    ui->setupUi(this);

    // 连接按钮
    connect(ui->btn_ok, &QPushButton::clicked, this, &Sampling::onOkClicked);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &Sampling::onCancelClicked);

    // 初始化工作线程和滤波器
    m_filters = new ct::Filters();
    m_filters->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_filters, &QObject::deleteLater);

    // 连接采样信号
    connect(this, &Sampling::DownSampling, m_filters, &ct::Filters::DownSampling, Qt::QueuedConnection);
    connect(this, &Sampling::UniformSampling, m_filters, &ct::Filters::UniformSampling, Qt::QueuedConnection);
    connect(this, &Sampling::RandomSampling, m_filters, &ct::Filters::RandomSampling, Qt::QueuedConnection);
    connect(this, &Sampling::ReSampling, m_filters, &ct::Filters::ReSampling, Qt::QueuedConnection);
    connect(this, &Sampling::SamplingSurfaceNormal, m_filters, &ct::Filters::SamplingSurfaceNormal, Qt::QueuedConnection);
    connect(this, &Sampling::NormalSpaceSampling, m_filters, &ct::Filters::NormalSpaceSampling, Qt::QueuedConnection);

    // 连接结果信号
    connect(m_filters, &ct::Filters::filterResult, this, &Sampling::samplingResult, Qt::QueuedConnection);

    m_thread.start();

    // 设置默认选择
    ui->cbox_method->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Sampling::~Sampling()
{
    m_thread.quit();
    if (!m_thread.wait(3000))
    {
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void Sampling::init()
{
    // 获取当前选中的点云
    if (!m_cloudtree) return;

    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud first!");
        reject();
        return;
    }

    // 只支持单个点云采样
    if (selected_clouds.size() > 1)
    {
        printW("Sampling only supports single cloud. Will process the first selected cloud.");
    }

    m_current_cloud = selected_clouds[0];
}

void Sampling::onOkClicked()
{
    if (!m_current_cloud || !m_cloudtree || !m_cloudview)
    {
        reject();
        return;
    }

    // 显示进度条
    m_cloudtree->showProgress("Sampling PointCloud...");
    m_cloudtree->bindWorker(m_filters);

    // 设置输入点云
    m_filters->setInputCloud(m_current_cloud);

    // 根据选择的方法执行采样
    switch (ui->cbox_method->currentIndex())
    {
        case METHOD_DOWNSAMPLING:
            emit DownSampling(ui->dspin_radius1->value());
            break;

        case METHOD_UNIFORMSAMPLING:
            emit UniformSampling(ui->dspin_radius2->value());
            break;

        case METHOD_RANDOMSAMPLING:
            emit RandomSampling(ui->spin_sample1->value(), ui->spin_seed1->value());
            break;

        case METHOD_RESAMPLING:
            emit ReSampling(ui->dspin_radius3->value(), ui->spin_order->value());
            break;

        case METHOD_SAMPLINGSURFACENORMAL:
            emit SamplingSurfaceNormal(ui->spin_sample2->value(), ui->spin_seed2->value(), ui->dspin_ratio->value());
            break;

        case METHOD_NORMALSPACESAMPLING:
            emit NormalSpaceSampling(ui->spin_sample3->value(), ui->spin_seed3->value(), ui->spin_bin->value());
            break;
    }
}

void Sampling::onCancelClicked()
{
    reject();
}

void Sampling::samplingResult(const ct::Cloud::Ptr& cloud, float time)
{
    if (!cloud || !m_current_cloud || !m_cloudtree || !m_cloudview)
    {
        m_cloudtree->closeProgress();
        reject();
        return;
    }

    // 打印完成信息
    printI(QString("Sampling completed in %1 ms. Original: %2 points -> Sampled: %3 points")
          .arg(time)
          .arg(m_current_cloud->size())
          .arg(cloud->size()));

    // 设置新点云的名称：原名称 + "-sampling"
    QString new_id = m_current_cloud->id() + "-sampling";
    cloud->setId(new_id);

    // 获取原点云的树节点
    QTreeWidgetItem* source_item = m_cloudtree->getItemById(m_current_cloud->id());
    if (!source_item)
    {
        printW("Failed to find source cloud in tree!");
        m_cloudtree->closeProgress();
        reject();
        return;
    }

    // 获取父节点（文件项）
    QTreeWidgetItem* parent_item = source_item->parent();
    if (!parent_item)
    {
        // 如果没有父节点，说明源点云是根节点，将新点云作为根节点添加
        m_cloudtree->insertCloud(cloud, nullptr, true);
    }
    else
    {
        // 将新点云添加到父节点下，与源点云同级
        m_cloudtree->insertCloud(cloud, parent_item, true);
    }

    // 在视图中显示新点云（保留原始颜色）
    m_cloudview->addPointCloud(cloud);
    // 不设置颜色，保留采样后的原始颜色
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);

    // 取消选中源点云，选中新生成的采样点云
    m_cloudtree->setCloudChecked(m_current_cloud, false);
    m_cloudtree->setCloudChecked(cloud, true);

    // 关闭进度条
    m_cloudtree->closeProgress();

    // 关闭对话框
    accept();
}
