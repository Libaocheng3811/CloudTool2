//
// Created by LBC on 2026/1/19.
//

#ifndef CLOUDTOOL2_DISTANCECALCULATOR_H
#define CLOUDTOOL2_DISTANCECALCULATOR_H

#include <QObject>
#include <vector>
#include "core/cloud.h"
#include "field_types.h"

namespace ct {
    class CT_EXPORT DistanceCalculator : public QObject {
        Q_OBJECT
    public:
        explicit DistanceCalculator(QObject *parent = nullptr);
        ~DistanceCalculator() override = default;

    public slots:
        /**
         * @brief 执行距离计算的核心槽函数
        * @param ref 参考点云 (Reference/Source)
        * @param comp 比较点云 (Compared/Target)
        * @param params 计算参数
        */
        void doCalculation(ct::Cloud::Ptr ref, ct::Cloud::Ptr comp, ct::DistanceParams params);

        /**
         * @brief 请求取消计算
         */
        void cancel() { m_is_canceled = true; }

    signals:
        void progress(int percent);

        /**
        * @brief 计算完成信号
        * @param distances 计算出的距离列表 (与 comp 点云的索引一一对应)
        * @param time 耗时 (秒)
        */
        void calculationFinished(const std::vector<float>& distances, float time);

        /**
         * @brief 错误或被取消
         */
        void calculationFailed(const QString& msg);

    private:
        // --- 具体的算法实现函数 ---
        // 最近邻距离
        void computeNearest(const ct::Cloud::Ptr& ref, const ct::Cloud::Ptr& comp, std::vector<float>& dists);

        // K近邻平均距离
        void computeKnnMean(const ct::Cloud::Ptr& ref, const ct::Cloud::Ptr& comp, int k, std::vector<float>& dists);

        // 半径平均距离
        void computeRadiusMean(const ct::Cloud::Ptr& ref, const ct::Cloud::Ptr& comp, double r, std::vector<float>& dists);

        // 点到网格 (预留)
        // void computeC2M(...);

    private:
        bool m_is_canceled{false};
    };
} // namespace ct

#endif //CLOUDTOOL2_DISTANCECALCULATOR_H
