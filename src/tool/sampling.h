//
// Created by LBC on 2026/3/24.
//

#ifndef CLOUDTOOL2_SAMPLING_H
#define CLOUDTOOL2_SAMPLING_H

#include "ui/base/customdialog.h"
#include "algorithm/filters.h"
#include <QThread>

namespace Ui
{
    class Sampling;
}

/**
 * @brief 采样对话框 - 模态对话框，执行采样后生成新点云挂到原点云下
 */
class Sampling : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Sampling(QWidget* parent = nullptr);
    ~Sampling() override;

    void init() override;

private slots:
    void onOkClicked();
    void onCancelClicked();

    void samplingResult(const ct::Cloud::Ptr& cloud, float time);

signals:
    void DownSampling(float radius);
    void UniformSampling(float radius);
    void RandomSampling(int sample, int seed);
    void ReSampling(float radius, int order);
    void SamplingSurfaceNormal(int sample, int seed, float ratio);
    void NormalSpaceSampling(int sample, int seed, int bin);

private:
    Ui::Sampling* ui;
    QThread m_thread;
    ct::Filters* m_filters;

    // 当前正在处理的点云
    ct::Cloud::Ptr m_current_cloud;

    enum SamplingMethod {
        METHOD_DOWNSAMPLING = 0,
        METHOD_UNIFORMSAMPLING = 1,
        METHOD_RANDOMSAMPLING = 2,
        METHOD_RESAMPLING = 3,
        METHOD_SAMPLINGSURFACENORMAL = 4,
        METHOD_NORMALSPACESAMPLING = 5
    };
};

#endif //CLOUDTOOL2_SAMPLING_H
