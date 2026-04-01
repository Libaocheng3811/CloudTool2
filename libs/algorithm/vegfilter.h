//
// Created by LBC on 2026/1/6.
//

#ifndef CLOUDTOOL2_VEGFILTER_H
#define CLOUDTOOL2_VEGFILTER_H

#include "core/cloud.h"
#include "core/exports.h"

#include <QObject>

#include <atomic>

namespace ct{
    // 植被指数类型
    enum class VegIndexType {
        ExG_ExR = 0, // Excess Green-Excess Red
        ExG, // Excess Green: 2g - r - b
        NGRDI,   // (g - r) / (g + r)
        CIVE     // 0.441r - 0.811g + 0.385b + 18.787
    };

    class CT_EXPORT VegetationFilter : public QObject{
        Q_OBJECT
    public:
        explicit VegetationFilter(QObject *parent = nullptr) : QObject(parent), m_cloud(nullptr) {}

        void setInputCloud(const Cloud::Ptr &cloud) {m_cloud = cloud; }

    signals:
        void progress(int percent);
        void filterResult(const Cloud::Ptr& veg_cloud, const Cloud::Ptr& non_veg_cloud, float time);

    public slots:

        /**
         * @brief 应用植被过滤器
         * @param index_type 采用的植被指数类型
         * @param threshold  植被指数过滤阈值
         */
        void applyVegFilter(int index_type, double threshold);

        void cancel() { m_is_canceled = true; }

    private:
        static double calculateOtsuThreshold(const std::vector<float>& values);

    private:
        Cloud::Ptr m_cloud;
        std::atomic<bool> m_is_canceled{false};
    };
} // namespace ct



#endif //CLOUDTOOL2_VEGFILTER_H
