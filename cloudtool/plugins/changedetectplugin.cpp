//
// Created by LBC on 2026/1/18.
//

// You may need to build the project (run Qt uic code generator) to get "ui_changedetectdialog.h" resolved

#include "changedetectplugin.h"
#include "ui_changedetectplugin.h"

#include <cmath>

// 辅助函数，配置点云
void setupResultCloud(ct::Cloud::Ptr cloud, const std::vector<float>& dists, const QString& idSuffix){
    if (!cloud || cloud->empty()) return;

    // 添加标量场 (Scalar Field)
    cloud->addScalarField("C2C_Nearest_Distance", dists);

    // 基于标量场进行伪彩色渲染 (Color Ramp)
    cloud->updateColorByField("C2C_Distance");

    // 设置其他属性
    cloud->setId(cloud->id() + idSuffix);
    cloud->setHasRGB(true); // 确保渲染器使用 RGB
}

ChangeDetectPlugin::ChangeDetectPlugin(QWidget *parent) :
        ct::CustomDialog(parent), ui(new Ui::ChangeDetectPlugin), m_thread(this) {
    ui->setupUi(this);

    qRegisterMetaType<ct::Cloud::Ptr>("ct::Cloud::Ptr");
    qRegisterMetaType<ct::Cloud::Ptr>("Cloud::Ptr");
    qRegisterMetaType<ct::DistanceParams>("DistanceParams");
    qRegisterMetaType<std::vector<float>>("std::vector<float>");

    m_calculator = new ct::DistanceCalculator(nullptr);
    m_calculator->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_calculator, &QObject::deleteLater);

    connect(this, &ChangeDetectPlugin::requestChangeDetect, m_calculator, &ct::DistanceCalculator::doCalculation);
    connect(m_calculator, &ct::DistanceCalculator::calculationFinished, this, &ChangeDetectPlugin::onFilterDone);

    connect(ui->btn_ok, &QPushButton::clicked, this, &ChangeDetectPlugin::onApply);
    connect(ui->btn_cancel, &QPushButton::clicked, this, &ChangeDetectPlugin::onCancel);

    m_thread.start();
}

ChangeDetectPlugin::~ChangeDetectPlugin() {
    m_thread.quit();
    if (!m_thread.wait(3000)) {
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void ChangeDetectPlugin::init() {
    ui->combo_reference->clear();
    ui->combo_compare->clear();

    // 获取可用点云
    std::vector<ct::Cloud::Ptr> allClouds = m_cloudtree->getAllClouds();
    if (allClouds.empty()){
        printW(QString("No clouds available"));
        ui->btn_ok->setEnabled(false);
        return;
    }

    for (const auto& cloud : allClouds){
        ui->combo_reference->addItem(cloud->id(), QVariant::fromValue(cloud));
        ui->combo_compare->addItem(cloud->id(), QVariant::fromValue(cloud));
    }

    // 智能默认选项
    std::vector<ct::Cloud::Ptr> selectedClouds = m_cloudtree->getSelectedClouds();
    if (selectedClouds.size() >= 2){
        // 认为第一个选择的是参考点云，第二个选择的是待比较点云
        int idxRef = ui->combo_reference->findText(selectedClouds[0]->id());
        int idxComp = ui->combo_compare->findText(selectedClouds[1]->id());

        if (idxRef >= 0) ui->combo_reference->setCurrentIndex(idxRef);
        if (idxComp >= 0) ui->combo_compare->setCurrentIndex(idxComp);
    }
    else if (selectedClouds.size() == 1 && allClouds.size() >= 2){
        // 如果仅选择一个点云，则默认选择该点云为参考点云
        int idxRef = ui->combo_reference->findText(selectedClouds[0]->id());
        ui->combo_reference->setCurrentIndex(idxRef);

        // 选择剩余点云为待比较点云
        for (int i = 0; i < ui->combo_compare->count(); ++i){
            if (i != idxRef){
                ui->combo_compare->setCurrentIndex(i);
                break;
            }
        }
    }

    ui->combo_method->clear();
    ui->combo_method->addItem("C2C - Nearest Neighbor", ct::DistanceParams::C2C_NEAREST);
    ui->combo_method->addItem("C2C - K-Mean", ct::DistanceParams::C2C_KNN_MEAN);
    ui->combo_method->addItem("C2C - Radius Mean", ct::DistanceParams::C2C_RADIUS_MEAN);

    // 连接信号
    connect(ui->combo_method, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ChangeDetectPlugin::onMethodChanged);
    // 默认选中第0页
    ui->stackedWidget->setCurrentIndex(0);
    ui->btn_ok->setEnabled(true);
}

void ChangeDetectPlugin::onApply() {
    m_refCloud = ui->combo_reference->currentData().value<ct::Cloud::Ptr>();
    m_compCloud = ui->combo_compare->currentData().value<ct::Cloud::Ptr>();

    if (!m_refCloud || !m_compCloud){
        printE(QString("Invalid clouds selected"));
        return;
    }

    if (m_refCloud == m_compCloud){
        printW(QString("Reference cloud and compare cloud are the same"));
        return;
    }
    ct::DistanceParams params;
    params.method = static_cast<ct::DistanceParams::Method>(ui->combo_method->currentData().toInt());

    // 根据选择的方法，只读取对应页面的控件值
    if (params.method == ct::DistanceParams::C2C_NEAREST) {
        // Nearest
        m_threshold = ui->dsb_nearestThreshold->value();
    }
    else {
        m_threshold = ui->dsb_meanThreshold->value();
        if (params.method == ct::DistanceParams::C2C_KNN_MEAN) {
            // 读取子栈 Page 0 的 K 值
            params.k_knn = ui->sb_Knn->value();
        }
        else if (params.method == ct::DistanceParams::C2C_RADIUS_MEAN) {
            // 读取子栈 Page 1 的 Radius 值
            params.radius = ui->dsb_radius->value();
        }
    }

    this->hide();
    m_cloudtree->showProgress("Calculating Distance...");
    m_cloudtree->bindWorker(m_calculator);

    emit requestChangeDetect(m_refCloud, m_compCloud, params);
}

void ChangeDetectPlugin::onCancel() {
    this->close();
}

void ChangeDetectPlugin::onMethodChanged(int index) {
    auto method = static_cast<ct::DistanceParams::Method>(ui->combo_method->itemData(index).toInt());

    if (method == ct::DistanceParams::C2C_NEAREST) {
        ui->stackedWidget->setCurrentIndex(0);
    }
    else {
        ui->stackedWidget->setCurrentIndex(1); // 切换到 Local Mean 页

        if (method == ct::DistanceParams::C2C_KNN_MEAN) {
            ui->lblParamName->setText("Neighbors (K):");
            ui->stackInput->setCurrentIndex(0); // 显示 SpinBox
        } else {
            ui->lblParamName->setText("Radius (m):");
            ui->stackInput->setCurrentIndex(1); // 显示 DoubleSpinBox
        }
    }
}

void ChangeDetectPlugin::onFilterDone(const std::vector<float>& distances, float time) {
    m_cloudtree->closeProgress();

    if (distances.size() != m_compCloud->size()){
        printW(QString("Calculation error: Result size mismatch."));
        return;
    }

    printI(QString("Distance calculation finished in %1 s").arg(time));

    ct::Cloud::Ptr changed(new ct::Cloud);
    ct::Cloud::Ptr unchanged(new ct::Cloud);

    std::vector<float> changed_dists;
    std::vector<float> unchanged_dists;

    changed->reserve(m_compCloud->size() / 10); // 假设变化点较少
    unchanged->reserve(m_compCloud->size());

    for(size_t i = 0; i < m_compCloud->size(); ++i){
        float d = distances[i];
        // 处理无效值 (如找不到对应点)
        if (std::isnan(d)) continue;

        if (d > m_threshold) {
            changed->push_back(m_compCloud->getPoint(i));
            changed_dists.push_back(d);
        } else {
            // <= 阈值 -> 未变化点
            // 只有当用户勾选导出时才保存，节省内存
            if (ui->chk_export->isChecked()) {
                unchanged->push_back(m_compCloud->getPoint(i));
                unchanged_dists.push_back(d);
            }
        }
    }

    std::vector<ct::Cloud::Ptr> results;

    if (!changed->empty()){
        changed->setId(m_compCloud->id());
        setupResultCloud(changed, changed_dists, "_Changed");
        results.push_back(changed);
    } else {
        printI("No changed points found (all within threshold).");
    }

    if (!unchanged->empty()) {
        unchanged->setId(m_compCloud->id());
        setupResultCloud(unchanged, unchanged_dists, "_Unchanged");
        // 可选：未变化点设为灰色，不显示距离颜色
        // unchanged->setCloudColor(ct::RGB{200, 200, 200});
        results.push_back(unchanged);
    }

    // 只有当有结果时才添加组
    if (!results.empty()) {
        QString groupName = m_compCloud->id() + "_ChangeDetect";
        m_cloudtree->addResultGroup(m_compCloud, results, groupName);
    }
    this->accept();
}
