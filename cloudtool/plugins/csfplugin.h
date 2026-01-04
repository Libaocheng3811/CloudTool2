//
// Created by LBC on 2026/1/4.
//

#ifndef CLOUDTOOL2_CSFPLUGIN_H
#define CLOUDTOOL2_CSFPLUGIN_H

#include <QObject>
#include <QThread>

#include "modules/csffilter.h"
#include "base/cloudtree.h"
#include "base/cloudview.h"

class CSFPlugin : public QObject {
    Q_OBJECT
public:
    CSFPlugin(ct::CloudTree* tree, ct::CloudView* view, QObject* parent = nullptr);
    ~CSFPlugin();

    void exec();

private slots:
    void onFilterDone(const ct::Cloud::Ptr &ground_cloud, const ct::Cloud::Ptr &off_ground_cloud, float time);

signals:
    void startFilter(bool bSloop, float dt, double thresh, double res, int rigid, int iter);

private:
    ct::CloudTree* m_tree;
    ct::CloudView* m_view;
    QThread m_thread;
    ct::CSFFilter* m_filter;

    ct::Cloud::Ptr m_source_cloud; // 原始点云
};

#endif //CLOUDTOOL2_CSFPLUGIN_H
