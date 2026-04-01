//
// Created by LBC on 2024/12/26.
//

#ifndef MODULES_FILTERS_H
#define MODULES_FILTERS_H

#include "core/exports.h"
#include "core/cloud.h"

#include <QObject>

#include <atomic>

#include <pcl/Vertices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>

namespace ct
{
    // pcl::ConditionBase 是 PCL（Point Cloud Library）中的一个模板类，通常用作点云条件的基类。
    typedef pcl::ConditionBase<PointXYZRGBN>    ConditionBase;
    typedef pcl::ConditionAnd<PointXYZRGBN>     ConditionAnd;
    typedef pcl::ConditionOr<PointXYZRGBN>      ConditionOr;
    typedef pcl::FieldComparison<PointXYZRGBN>  FieldComparison;
    typedef pcl::ComparisonOps::CompareOp       CompareOp;

    class CT_EXPORT Filters : public QObject
    {
        Q_OBJECT
    public:
        explicit Filters(QObject* parent = nullptr)
            : QObject(parent),
            cloud_(nullptr),
            negative_(false)
            {}

        void setInputCloud(const Cloud::Ptr &cloud) {cloud_ = cloud; };

        /**
         * @brief 设置是应用点过滤的常规条件，还是应用倒置条件
         */
        void setNegative(bool negative) {negative_ = negative; };

    private:
        Cloud::Ptr cloud_;
        // negative_ 的作用是控制降采样滤波操作时是否反向操作
        // negative_为true时，保留滤波器选出的保留点，否则，为false时，保留将要被滤波器移除的点
        bool negative_;
        std::atomic<bool> m_is_canceled{false};

    signals:

        /**
         * @brief 点云滤波的结果
         */
        void filterResult(const Cloud::Ptr &cloud, float time);

        void progress(int percent);

    public slots:

        /* filter */
        /**
         * @brief 直通滤波器
         * @param field_name 提供要用于过滤数据的字段的名称
         * @param limit_min 为过滤数据的字段设置数值限制
         * @param limit_max 为过滤数据的字段设置数值限制
         * 设置某个维度进行滤波，设置该维度的阈值，超过阈值的点云删除，在阈值范围内的点云保留
         */
        void PassThrough(const std::string &field_name, float limit_min, float limit_max);

        /**
         * @brief 体素栅格滤波， 基于体素网格下采样的滤波器 ，在给定的 PointCloud 上组装一个本地3D网格，并对数据进行下采样+过滤
         * @param lx ly lz 设置体素网格叶大小
         * 基本原理是将点云数据空间划分为一系列大小相同的三维体素栅格。对于每个体素栅格，算法会计算该栅格内所有点的重心或其他统计量（如平均值、中位数等），
         * 并选择一个点（通常是重心点）来代表该栅格内的所有点。这样，每个体素栅格就只用一个点来表示，从而大大减少了点云的数据量。
         */
        void VoxelGrid(float lx, float ly, float lz);

        /**
         * @brief 体素栅格滤波， 基于体素网格下采样的滤波器，在给定的 PointCloud 上组装一个本地3D网格，并对数据进行下采样+过滤
         * @param lx ly lz 设置体素网格叶大小
         * ApproximateVoxelGrid 是 VoxelGrid 的一个变种，它的区别在于体素化方法上更加近似，依据栅格的中心点来代表该栅格的点云。
         * 而不是精确计算每个体素内的代表点。它通过减少计算的精度来提高速度，从而降低处理的时间和资源消耗。
         */
        void ApproximateVoxelGrid(float lx, float ly, float lz);

        /**
         * @brief 统计滤波器，基于统计学的离群点移除滤波器
         * @param nr_k 设置用于平均距离估计的最近邻点数量
         * @param stddev_mult 设置距离阈值计算的标准偏差乘数
         * 对于每个点，滤波器会计算它与邻域内所有点的距离的平均值（称为 mean distance），以及这些距离的标准差。
         * 如果一个点的平均距离与其邻域的其他点相比显著较大（即超过了 stddev_mult 倍的标准差），该点就会被认为是离群点。
         */
        void StatisticalOutlierRemoval(int nr_k, double stddev_mult);

        /**
         * @brief 离群点滤波
         * @param radius 球体的搜索半径，确定将搜索半径内的点作为邻近点
         * @param min_pts 设置为了被分类为内点而需要存在的邻居数量，也就是只保留位于查询点的搜索半径内的点的数目不低于该值的查询点。
         * 最终搜索半径内邻近点数量少于min_pts的查询点被认为是离群点，需要滤除
         */
        void RadiusOutlierRemoval(double radius, int min_pts);

        /**
         * @brief 条件滤波器
         * @param con 设置过滤器将使用的条件
         * 这个滤波器用于删除点云中不符合指定的一个或多个条件的数据点。类似于直通滤波
         * 如果设置不删除不满足条件的数据点，默认情况下，点云中不满足条件的数据点的每个字段的值都被设置为 NAN，可以该默认值可以设置为其它值
         */
        void ConditionalRemoval(ConditionBase::Ptr con);

        /**
         * @brief 在给定的 PointCloud 上组装一个本地 2D 网格，并对数据进行下采样
         * @param resolution 设置网格分辨率
         * 通过将三维点云在X-Y平面划分网格，然后寻找每个网格中最小的z点云代表该网格。
         */
        void GridMinimun(const float resolution);

        /**
         * @brief 通过消除局部最大值来对云进行下采样
         * @param radius 设置用于确定是否为局部最大值的半径
         * 依次判断并删除，每个点云点的邻域半径范围内局部最大的点。
         * 在每个点云点的邻域半径内，查找z值距离该点云点最远的点删除该点，并依次遍历所有点云点；得到结果点云
         */
        void LocalMaximum(float radius);

        /**
         * @brief 去除出现在边缘不连续的鬼点
         * @param threshold 设置阴影点拒绝的阈值
         */
        void ShadowPoints(float threshold);

        /* sample */
        /**
        * @brief 体素网格降采样，使用 VoxelGrid 滤波器对点云进行下采样
        * @param radius 体素大小（Voxel leaf size），单位与点云坐标单位一致
        * @note 在 3D 空间创建体素网格，每个体素内保留一个代表点（通常为重心）
        */
        void DownSampling(float radius);

        /**
         * @brief 均匀采样，使用 UniformSampling 滤波器在 3D 空间网格上进行下采样
         * @param radius 搜索半径，用于确定 3D 空间网格的大小
         * @note 与 DownSampling 类似，但使用不同的采样策略，基于网格中心进行采样
         */
        void UniformSampling(float radius);

        /**
         * @brief 随机采样，从点云中随机抽取指定数量的点
         * @param sample 采样点数（保留的点数）
         * @param seed 随机种子，用于可重复的随机采样
         * @note 使用均匀概率进行随机抽样，适合快速预览或测试
         */
        void RandomSampling(int sample, int seed);

        /**
         * @brief 移动最小二乘重采样，用于点云平滑和法线计算
         * @param radius 搜索半径，用于查找 k 近邻点
         * @param polynomial_order 多项式阶数（1-5），用于局部表面拟合
         * @note 使用 MLS (Moving Least Squares) 算法平滑点云并重新采样，可同时计算法线
         */
        void ReSampling(float radius, int polynomial_order);

        /**
         * @brief 表面法线空间采样，在法线方向空间均匀采样点云
         * @param sample 每个网格中的最大样本数
         * @param seed 随机种子
         * @param ratio 每个网格中要采样的点的比率（0-1）
         * @note 将空间划分为网格，考虑表面法线方向进行采样，保留几何特征
         */
        void SamplingSurfaceNormal(int sample, int seed, float ratio);

        /**
         * @brief 法线空间采样，在法线方向空间均匀采样以保留特征
         * @param sample 采样点数
         * @param seed 随机种子
         * @param bin x,y,z 方向的 bin 数量（法线空间分割数）
         * @note 在法线方向空间进行均匀采样，更好地保留点云的几何特征
         */
        void NormalSpaceSampling(int sample, int seed, int bin);

        /**
         * @brief 取消操作
         */
        void cancel() { m_is_canceled = true;}
    };
}


#endif //MODULES_FILTERS_H
