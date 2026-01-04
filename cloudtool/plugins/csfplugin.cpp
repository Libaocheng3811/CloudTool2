//
// Created by LBC on 2026/1/4.
//

#include "csfplugin.h"
#include "csfdialog.h"


CSFPlugin::CSFPlugin(ct::CloudTree *tree, ct::CloudView *view, QObject *parent)
    : QObject(parent), m_tree(tree), m_view(view){
    // 初始化后台线程
    m_filter = new ct::CSFFilter;
    m_filter->moveToThread(&m_thread);

    connect(&m_thread, &QThread::finished, m_filter, &QObject::deleteLater);
    connect(this, &CSFPlugin::startFilter, m_filter, &ct::CSFFilter::applyCSF);
    connect(m_filter, &ct::CSFFilter::filterResult, this, &CSFPlugin::onFilterDone);

    m_thread.start();
}

CSFPlugin::~CSFPlugin() {
    m_thread.quit();
    m_thread.wait();
}

void CSFPlugin::exec() {
    auto selection = m_tree->getSelectedClouds();
    if (selection.empty()) return;

    m_source_cloud = selection.front();

    // 阻塞式窗口
    CSFDialog dlg;
    if (dlg.exec() != QDialog::Accepted) return;

    //获取参数
    CSFParams p = dlg.getParams();

    m_filter->setInputCloud(m_source_cloud);
    m_tree->showProgressBar();

    emit startFilter(p.bsloopSmooth, p.time_step, p.class_threshold,
                     p.cloth_resolution, p.rigidness, p.iterations);
}

void CSFPlugin::onFilterDone(const ct::Cloud::Ptr &ground_cloud, const ct::Cloud::Ptr &off_ground_cloud,
                             float time)
{
    m_tree->closeProgressBar();

    ground_cloud->setId(m_source_cloud->id() + "_ground");
    off_ground_cloud->setId(m_source_cloud->id() + "_off_ground");

    //设置颜色
    // ct::RGB{0,255,0}创建临时对象
    ground_cloud->setCloudColor(ct::RGB{0,255,0}); // 绿地
    off_ground_cloud->setCloudColor(ct::RGB{255,0,0}); // 红房

    m_tree->appendCloud(m_source_cloud, ground_cloud);
    m_tree->appendCloud(m_source_cloud, off_ground_cloud);

    // 隐藏原始点云
    m_tree->setCloudChecked(m_source_cloud, false);

}