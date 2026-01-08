//
// Created by LBC on 2024/12/25.
//

#ifndef TOOL_FILTERS_H
#define TOOL_FILTERS_H

#include "widgets/customdock.h"
#include "modules/filters.h"
#include <QThread>


QT_BEGIN_NAMESPACE
namespace Ui {
    class Filters;
}
QT_END_NAMESPACE

class Filters : public ct::CustomDock {
Q_OBJECT

public:
    explicit Filters(QWidget *parent = nullptr);

    ~Filters() override;

    void preview();

    // 将过滤后的点云（保留下的点云）添加到视图树中
    void add();

    void apply();

    virtual void reset();

signals:
    void PassThrough(const std::string &field_name, float limit_min, float limit_max);
    void VoxelGrid(float lx, float ly, float lz);
    void ApproximateVoxelGrid(float lx, float ly, float lz);
    void StatisticalOutlierRemoval(int nr_k, double stddev_mult);
    void RadiusOutlierRemoval(double radius, int min_pts);
    void ConditionalRemoval(ct::ConditionBase::Ptr con);
    void GridMinimum(const float resolution);
    void LocalMaximum(float radius);
    void ShadowPoints(float threshold);

public slots:
    void filterResult(const ct::Cloud::Ptr & cloud, float time);

private:
    ct::ConditionBase::Ptr getCondition();
    void getRange(int index);

private:
    Ui::Filters *ui;
    QThread m_thread;
    // 滤波器
    ct::Filters *m_filters;
    std::map<QString, ct::Cloud::Ptr> m_filter_map;
};


#endif //TOOL_FILTERS_H
