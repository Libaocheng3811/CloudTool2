//
// Created by LBC on 2026/1/4.
//

#ifndef CLOUDTOOL2_CSFFILTER_H
#define CLOUDTOOL2_CSFFILTER_H

#include "base/exports.h"
#include "base/cloud.h"

#include <QObject>

#include <atomic>

#include "3rdparty/CSF/src/CSF.h"
#include "3rdparty/CSF/src/point_cloud.h"

namespace ct{
    class CT_EXPORT CSFFilter : public QObject{
        Q_OBJECT
    public:
        explicit CSFFilter(QObject *parent = nullptr)
            : QObject(parent), cloud_(nullptr){
        }
        void setInputCloud(const Cloud::Ptr &cloud) {cloud_ = cloud; }

    signals:
        /**
         * @brief 地面滤波结果
         * @param ground_cloud 滤波后的地面点云
         * @param off_ground_cloud 滤波后的非地面点云
         * @param time 耗时
         */
        void filterResult(const Cloud::Ptr& ground_cloud, const Cloud::Ptr& off_ground_cloud, float time);

        void progress(int percent);

    public slots:
        /**
         * @brief 应用CSF滤波
         * @param bSloopSmooth 是否进行坡度平滑
         * @param time_step 时间步长
         * @param class_threshold 分类阈值
         * @param cloth_resolution 布料分辨率
         * @param rigidness 布料硬度（1,2,3）
         * @param iterations 迭代次数
         */
        void applyCSF(bool bSloopSmooth, float time_step, double class_threshold, double cloth_resolution,
                      int rigidness, int iterations);

        void cancel() { m_is_canceled = true;}

    private:
        Cloud::Ptr cloud_;
        std::atomic<bool> m_is_canceled{false};
    };
} // namespace ct


#endif //CLOUDTOOL2_CSFFILTER_H
