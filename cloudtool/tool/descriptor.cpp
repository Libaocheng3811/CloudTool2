//
// Created by LBC on 2025/1/10.
//

#include "descriptor.h"
#include "ui_descriptor.h"

#define DESCRIPTOR_TYPE_PFHEstimation                               (0)
#define DESCRIPTOR_TYPE_FPFHEstimation                              (1)
#define DESCRIPTOR_TYPE_VFHEstimation                               (2)
#define DESCRIPTOR_TYPE_ESFEstimation                               (3)
#define DESCRIPTOR_TYPE_GASDEstimation                              (4)
#define DESCRIPTOR_TYPE_GASDColorEstimation                         (5)
#define DESCRIPTOR_TYPE_RSDEstimation                               (6)
#define DESCRIPTOR_TYPE_GRSDEstimation                              (7)
#define DESCRIPTOR_TYPE_CRHEstimation                               (8)
#define DESCRIPTOR_TYPE_CVFHEstimation                              (9)
#define DESCRIPTOR_TYPE_ShapeContext3DEstimation                    (10)
#define DESCRIPTOR_TYPE_SHOTEstimation                              (11)
#define DESCRIPTOR_TYPE_SHOTColorEstimation                         (12)
#define DESCRIPTOR_TYPE_UniqueShapeContext                          (13)
#define DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation          (14)
#define DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation          (15)
#define DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation           (16)


Descriptor::Descriptor(QWidget *parent) :
        CustomDock(parent), ui(new Ui::Descriptor),
        m_thread(this),
        m_plotter(new pcl::visualization::PCLPlotter) {
    ui->setupUi(this);

    // 注册重载类型，
    // 第一行代码注册了 ct::FeatureType::Ptr 的左值引用类型 ("FeatureType::Ptr &")，
    // 第二行代码注册了 ct::FeatureType::Ptr 的类型 ("FeatureType::Ptr")，这意味着可以直接传递指针类型。
    qRegisterMetaType<ct::FeatureType::Ptr>("FeatureType::Ptr &");
    qRegisterMetaType<ct::FeatureType::Ptr>("FeatureType::Ptr");
    qRegisterMetaType<ct::ReferenceFrame::Ptr>("ReferenceFrame::Ptr &");
    qRegisterMetaType<ct::ReferenceFrame::Ptr>("ReferenceFrame::Ptr");

    connect(ui->btn_apply, &QPushButton::clicked, this, &Descriptor::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Descriptor::reset);

    m_features = new ct::Features;
    m_features->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_features, &QObject::deleteLater);
    connect(this, &Descriptor::PFHEstimation, m_features, &ct::Features::PFHEstimation);
    connect(this, &Descriptor::FPFHEstimation, m_features, &ct::Features::FPFHEstimation);
    connect(this, &Descriptor::VFHEstimation, m_features, &ct::Features::VFHEstimation);
    connect(this, &Descriptor::ESFEstimation, m_features, &ct::Features::ESFEstimation);
    connect(this, &Descriptor::GASDEstimation, m_features, &ct::Features::GASDEstimation);
    connect(this, &Descriptor::GASDColorEstimation, m_features, &ct::Features::GASDColorEstimation);
    connect(this, &Descriptor::RSDEstimation, m_features, &ct::Features::RSDEstimation);
    connect(this, &Descriptor::GRSDEstimation, m_features, &ct::Features::GRSDEstimation);
    connect(this, &Descriptor::CRHEstimation, m_features, &ct::Features::CRHEstimation);
    connect(this, &Descriptor::CVFHEstimation, m_features, &ct::Features::CVFHEstimation);
    connect(this, &Descriptor::ShapeContext3DEstimation, m_features, &ct::Features::ShapeContext3DEstimation);
    connect(this, &Descriptor::SHOTEstimation, m_features, &ct::Features::SHOTEstimation);
    connect(this, &Descriptor::SHOTColorEstimation, m_features, &ct::Features::SHOTColorEstimation);
    connect(this, &Descriptor::UniqueShapeContext, m_features, &ct::Features::UniqueShapeContext);
    connect(this, &Descriptor::BOARDLocalReferenceFrameEstimation, m_features,
            &ct::Features::BOARDLocalReferenceFrameEstimation);
    connect(this, &Descriptor::FLARELocalReferenceFrameEstimation, m_features,
            &ct::Features::FLARELocalReferenceFrameEstimation);
    connect(this, &Descriptor::SHOTLocalReferenceFrameEstimation, m_features,
            &ct::Features::SHOTLocalReferenceFrameEstimation);
    connect(m_features, &ct::Features::featureResult, this, &Descriptor::featureResult);
    connect(m_features, &ct::Features::lrfResult, this, &Descriptor::lrfResult);

    // 线程启动
    m_thread.start();

    connect(ui->cbox_feature, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index) {
        // 如果index = 0，意味着选择了特征类型，则显示特征类型下拉框
        if (index == 0) {
            ui->cbox_type->show();
            ui->cbox_lrf->hide();
            // 显示特征类型对应的停靠面板
            ui->stackedWidget->setCurrentIndex(ui->cbox_type->currentIndex());
        } else {
            ui->cbox_type->hide();
            ui->cbox_lrf->show();
            // 显示特征类型对应的停靠面板
            ui->stackedWidget->setCurrentIndex(ui->cbox_lrf->currentIndex() + 14);
        }
    });

    connect(ui->cbox_lrf, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index) {
        ui->stackedWidget->setCurrentIndex(index + 14);
    });

    // 初始情况下，显示特征类型下拉框，隐藏local reference frame下拉框
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_lrf->hide();

}

Descriptor::~Descriptor() {
    // 结束线程的事件循环，但不会立即终止线程，需要等待线程完成任务后再退出。
    m_thread.quit();
    if (!m_thread.wait(3000)) {
        m_thread.terminate();
        m_thread.wait();
    }
    delete ui;
}

void Descriptor::preview() {
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty()) {
        printW("Please select at least one cloud!");
        return;
    }

    // clearPlots() 方法的作用是清除当前 PCLPlotter 窗口中所有的图表或绘图内容
    m_plotter->clearPlots();
    if (ui->check_show_histogram->isChecked())
        m_plotter.reset(new pcl::visualization::PCLPlotter);
    for (auto &cloud: selected_clouds) {
        if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0) {
            printW("Parameter set error!");
            return;
        }
        m_features->setInputCloud(cloud);
        m_features->setKSearch(ui->spin_k->value());
        m_features->setRadiusSearch(ui->dspin_r->value());
        switch (ui->cbox_type->currentIndex()) {
            case DESCRIPTOR_TYPE_PFHEstimation:
                m_cloudview->showInfo("PFHEstimation", 1);
                emit PFHEstimation();
                break;
            case DESCRIPTOR_TYPE_FPFHEstimation:
                m_cloudview->showInfo("FPFHEstimation", 1);
                emit FPFHEstimation();
                break;
            case DESCRIPTOR_TYPE_VFHEstimation:
                m_cloudview->showInfo("VFHEstimation", 1);
                emit VFHEstimation(
                Eigen::Vector3f(ui->dspin_vpx1->value(), ui->dspin_vpy1->value(), ui->dspin_vpz1->value()));
                break;
            case DESCRIPTOR_TYPE_ESFEstimation:
                m_cloudview->showInfo("ESFEstimation", 1);
                emit ESFEstimation();
                break;
            case DESCRIPTOR_TYPE_GASDEstimation:
                m_cloudview->showInfo("GASDEstimation", 1);
                emit GASDEstimation(
                Eigen::Vector3f(ui->dspin_vx1->value(), ui->dspin_vy1->value(), ui->dspin_vz1->value()),
                ui->spin_shgs1->value(), ui->spin_shs1->value(), ui->cbox_interp1->currentIndex());
                break;
            case DESCRIPTOR_TYPE_GASDColorEstimation:
                m_cloudview->showInfo("GASDColorEstimation", 1);
                emit GASDColorEstimation(
                Eigen::Vector3f(ui->dspin_vx2->value(), ui->dspin_vy2->value(), ui->dspin_vz2->value()),
                ui->spin_shgs2->value(), ui->spin_shs2->value(), ui->cbox_interp2->currentIndex(),
                ui->spin_chgs->value(), ui->spin_chs->value(), ui->cbox_cinterp->currentIndex());
                break;
            case DESCRIPTOR_TYPE_RSDEstimation:
                m_cloudview->showInfo("RSDEstimation", 1);
                emit RSDEstimation(ui->spin_nr_subdiv->value(), ui->dspin_plane_radius->value());
                break;
            case DESCRIPTOR_TYPE_GRSDEstimation:
                m_cloudview->showInfo("GRSDEstimation", 1);
                emit GRSDEstimation();
                break;
            case DESCRIPTOR_TYPE_CRHEstimation:
                m_cloudview->showInfo("CRHEstimation", 1);
                emit CRHEstimation(
                Eigen::Vector3f(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value()));
                break;
            case DESCRIPTOR_TYPE_CVFHEstimation:
                m_cloudview->showInfo("CVFHEstimation", 1);
                emit CVFHEstimation(
                Eigen::Vector3f(ui->dspin_vpx2->value(), ui->dspin_vpy2->value(), ui->dspin_vpz2->value()),
                ui->dspin_rn->value(), ui->dspin_d1->value(), ui->dspin_d2->value(), ui->dspin_d3->value(),
                ui->spin_min->value(), ui->check_normalize->isChecked());
                break;
            case DESCRIPTOR_TYPE_ShapeContext3DEstimation:
                m_cloudview->showInfo("ShapeContext3DEstimation", 1);
                emit ShapeContext3DEstimation(ui->dspin_min_r->value(), ui->dspin_r2->value());
                break;
            case DESCRIPTOR_TYPE_SHOTEstimation:
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame First!");
                    return;
                }
                m_cloudview->showInfo("SHOTEstimation", 1);
                emit SHOTEstimation(m_lrf_map.find(cloud->id())->second, ui->dspin_lrf1->value());
                break;
            case DESCRIPTOR_TYPE_SHOTColorEstimation:
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame First!");
                    return;
                }
                m_cloudview->showInfo("SHOTColorEstimation", 1);
                emit SHOTColorEstimation(m_lrf_map.find(cloud->id())->second, ui->dspin_lrf2->value());
                break;
            case DESCRIPTOR_TYPE_UniqueShapeContext:
                if (m_lrf_map.find(cloud->id()) == m_lrf_map.end()) {
                    printW("Please Estimation LocalReferenceFrame first!");
                    return;
                }
                m_cloudview->showInfo("UniqueShapeContext", 1);
                emit UniqueShapeContext(m_lrf_map.find(cloud->id())->second, ui->dspin_min_r2->value(),
                                        ui->dspin_p_r->value(),
                                        ui->dspin_l_r->value());
                break;
            case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation:
                m_cloudview->showInfo("BOARDLocalReferenceFrameEstimation", 1);
                emit BOARDLocalReferenceFrameEstimation(ui->dspin_t_r1->value(), ui->check_find_holes->isChecked(),
                                                        ui->dspin_m_t1->value(),
                                                        ui->spin_size->value(), ui->dspin_h_t1->value(),
                                                        ui->dspin_s_t1->value());
                break;
            case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation:
                m_cloudview->showInfo("FLARELocalReferenceFrameEstimation", 1);
                emit FLARELocalReferenceFrameEstimation(ui->dspin_t_r2->value(), ui->dspin_m_t2->value(),
                                                        ui->spin_m_n2->value(), ui->spin_m_t2->value());
                break;
            case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation:
                m_cloudview->showInfo("SHOTLocalReferenceFrameEstimation", 1);
                emit SHOTLocalReferenceFrameEstimation();
                break;
        }
        m_cloudtree->showProgressBar();
    }
}

void Descriptor::reset() {

    /**
     * @breif 这里涉及一个重要的知识点，关于m_plotter.reset，m_plotter->close()中的.运算符和->运算符的区别。
     * 首先，智能指针 std::shared_ptr<T> 本质上是一个对象，不是普通指针。它是一个封装了指针的类
     * .(点运算符)用于对象本身（普通变量）； ->(箭头运算符)用于指针指向的对象（即(*ptr).method()）。
     * ptr.reset() 调用的是 shared_ptr 本身的 reset() 方法。
     * m_plotter->close();  调用 PCLPlotter 的 close() 方法。
     * 但是，但是，但是，在 C++ 中，普通指针（如 int* p）一般只能使用 -> 运算符来访问指针指向的对象的成员，
     * 几乎所有情况下，你不能直接对 p 本身使用 . 来调用 int 的成员，因为 int 不是类，没有成员函数。
     */
    // 重置智能指针m_plotter，释放它所管理的旧对象（如果有的话），并分配一个新的 PCLPlotter 对象。
    m_plotter.reset(new pcl::visualization::PCLPlotter);
    // close 方法用于关闭 PCLPlotter 对象所显示的窗口或界面
    m_plotter->close();
    m_plotter = nullptr;
    m_descriptor_map.clear();
    m_lrf_map.clear();
    m_cloudview->clearInfo();
}

void Descriptor::featureResult(const QString &id, const ct::FeatureType::Ptr &feature, float time) {
    m_descriptor_map[id] = feature;
    m_cloudtree->closeProgressBar();
    switch (ui->cbox_type->currentIndex()) {
        case DESCRIPTOR_TYPE_PFHEstimation:
            printI(QString("Estimate cloud[id:%1] PFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("PFHEstimation");
            // 显示特征直方图
            m_plotter->addFeatureHistogram(*feature->pfh, 1000);
            break;
        case DESCRIPTOR_TYPE_FPFHEstimation:
            printI(QString("Estimate cloud[id:%1] FPFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("FPFHEstimation");
            m_plotter->addFeatureHistogram(*feature->fpfh, 1000);
            break;
        case DESCRIPTOR_TYPE_VFHEstimation:
            printI(QString("Estimate cloud[id:%1] VFHFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("VFHEstimation");
            m_plotter->addFeatureHistogram(*feature->vfh, 1000);
            break;
        case DESCRIPTOR_TYPE_ESFEstimation:
            printI(QString("Estimate cloud[id:%1] ESFFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("ESFEstimation");
            m_plotter->addFeatureHistogram(*feature->esf, 1000);
            break;
        case DESCRIPTOR_TYPE_GASDEstimation:
            printI(QString("Estimate cloud[id:%1] GASDFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GASDEstimation");
            m_plotter->addFeatureHistogram(*feature->gasd, 1000);
            break;
        case DESCRIPTOR_TYPE_GASDColorEstimation:
            printI(QString("Estimate cloud[id:%1] GASDColorFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GASDColorEstimation");
            m_plotter->addFeatureHistogram(*feature->gasdc, 1000);
            break;
        case DESCRIPTOR_TYPE_RSDEstimation:
            printI(QString("Estimate cloud[id:%1] RSDEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("RSDEstimation");
            // m_plotter->addFeatureHistogram(*feature->rsd, 1000);
            break;
        case DESCRIPTOR_TYPE_GRSDEstimation:
            printI(QString("Estimate cloud[id:%1] GRSDEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("GRSDEstimation");
            m_plotter->addFeatureHistogram(*feature->grsd, 1000);
            break;
        case DESCRIPTOR_TYPE_CRHEstimation:
            printI(QString("Estimate cloud[id:%1] CRHEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("CRHEstimation");
            m_plotter->addFeatureHistogram(*feature->crh, 1000);
            break;
        case DESCRIPTOR_TYPE_CVFHEstimation:
            printI(QString("Estimate cloud[id:%1] CVFHEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("CVFHEstimation");
            m_plotter->addFeatureHistogram(*feature->vfh, 1000);
            break;
        case DESCRIPTOR_TYPE_ShapeContext3DEstimation:
            printI(QString("Estimate cloud[id:%1] ShapeContext3DEstimation done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("ShapeContext3DEstimation");
            // m_plotter->addFeatureHistogram(*feature->sc3d, 1000);
            break;
        case DESCRIPTOR_TYPE_SHOTEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("SHOTEstimation");
            // m_plotter->addFeatureHistogram(*feature->shot, 1000);
            break;
        case DESCRIPTOR_TYPE_SHOTColorEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTColorFeature done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("SHOTColorEstimation");
            // m_plotter->addFeatureHistogram(*feature->shotc, 1000);
            break;
        case DESCRIPTOR_TYPE_UniqueShapeContext:
            printI(QString("Estimate cloud[id:%1] UniqueShapeContext done, take time %2 ms.").arg(id).arg(time));
            m_plotter->setTitle("UniqueShapeContext");
            // m_plotter->addFeatureHistogram(*feature->usc, 1000);
            break;
    }
    if (ui->check_show_histogram->isChecked())
    {
        // 在窗口中计算出一个点的位置，并设置为直方图窗口的位置
        QPoint pos = m_cloudview->mapToGlobal(QPoint((m_cloudview->width() - 640) / 2, (m_cloudview->height() - 200) / 2));
        m_plotter->setWindowPosition(pos.x(), pos.y());
        // 绘制显示直方图
        m_plotter->plot();
    }
}

void Descriptor::lrfResult(const QString &id, const ct::ReferenceFrame::Ptr& cloud, float time)
{
    switch(ui->cbox_type->currentIndex())
    {
        case DESCRIPTOR_TYPE_BOARDLocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] BOARDLocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
        case DESCRIPTOR_TYPE_FLARELocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] FLARELocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
        case DESCRIPTOR_TYPE_SHOTLocalReferenceFrameEstimation:
            printI(QString("Estimate cloud[id:%1] SHOTLocalReferenceFrame done, take time %2 ms.").arg(id).arg(time));
            break;
    }
    m_lrf_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}