//
// Created by LBC on 2026/1/4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_csfplugin.h" resolved

#include "csfplugin.h"
#include "ui_csfplugin.h"

CSFPlugin::CSFPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::CSFPlugin), m_thread(this) {
    ui->setupUi(this);

    qRegisterMetaType<ct::Cloud::Ptr>("ct::Cloud::Ptr");
    qRegisterMetaType<ct::Cloud::Ptr>("Cloud::Ptr");

    m_filter = new ct::CSFFilter(nullptr);
    m_filter->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_filter, &QObject::deleteLater);

    // 信号连接
    connect(this, &CSFPlugin::requestCSF, m_filter, &ct::CSFFilter::applyCSF);
    connect(m_filter, &ct::CSFFilter::filterResult, this, &CSFPlugin::onFilterDone);

    // ui按钮
    connect(ui->btnOk, &QPushButton::clicked, this, &CSFPlugin::onApply);
    connect(ui->btnCancel, &QPushButton::clicked, this, &CSFPlugin::onCancel);

    m_thread.start();
}

CSFPlugin::~CSFPlugin() {
    m_thread.quit();
    if (!m_thread.wait(3000)){
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void CSFPlugin::init(){
    auto selection = m_cloudtree->getSelectedClouds();
    if (selection.empty()){
        printW("Please select at least one cloud.");
        ui->btnOk->setEnabled(false);
        return;
    }

    m_cloud = selection.front();
    m_filter->setInputCloud(m_cloud);
    ui->btnOk->setEnabled(true);
}

void CSFPlugin::onApply() {
    if (!m_cloud) return;

    //获取参数
    bool smooth = ui->m_chkSlopeProcessing->isChecked();
    double res = ui->m_dsResolution->value();
    double thresh = ui->m_dsThreshold->value();
    int iter = ui->m_spIterations->value();

    int rigidness = 2; // 默认为relief
    if (ui->m_rbFlat->isChecked()) rigidness = 3;
    else if (ui->m_rbRelief->isChecked()) rigidness = 2;
    else rigidness = 1;

    this->hide();
    QCoreApplication::processEvents();

    m_cloudtree->showProgress("Running Cloth Simulation Filter...");
    m_cloudtree->bindWorker(m_filter);

    emit requestCSF(smooth, 0.65f, thresh, res, rigidness, iter);
}

void CSFPlugin::onCancel() {
    this->close();
}

void CSFPlugin::onFilterDone(const ct::Cloud::Ptr& ground_cloud, const ct::Cloud::Ptr& off_ground_cloud, float time) {
    m_cloudtree->closeProgress();
    printI(QString("CSF Finished in %1 s").arg(time));

    ground_cloud->setId(m_cloud->id() + "_ground");
    off_ground_cloud->setId(m_cloud->id() + "_off_ground");

    if (!m_cloud->hasRGB()){
        // 如果没有RGB信息，手动赋色
        ground_cloud->setCloudColor(ct::RGB{0, 255, 0}); // Green
        off_ground_cloud->setCloudColor(ct::RGB{255, 0, 0}); // Red
    }

    m_cloudtree->setCloudChecked(m_cloud, false);

    m_cloudtree->appendCloud(m_cloud, ground_cloud);
    m_cloudtree->appendCloud(m_cloud, off_ground_cloud);

    this->accept();
}