//
// Created by LBC on 2024/12/26.
//

#include "modules/filters.h"

#include <pcl/console/time.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/impl/local_maximum.hpp>
#include <pcl/filters/impl/project_inliers.hpp>

namespace ct
{
    void Filters::PassThrough(const std::string& field_name, float limit_min, float limit_max)
    {
        // 计时器初始化
        TicToc time;
        // tic()方法开始计时，用于测量函数执行时间
        time.tic();
        // 创建过滤后的点云对象
        Cloud::Ptr cloud_filtered(new Cloud);
        // 设置cloud_filtered 的ID为原始点云对象ID，保持一致性
        cloud_filtered->setId(cloud_->id());

        // 创建一个PassThrough过滤器对象，用于直通滤波
        pcl::PassThrough<PointXYZRGBN> pfilter;
        // 设置输入点云
        pfilter.setInputCloud(cloud_);
        // 指定要过滤的字段名称，比如x, y, z, rgb等
        pfilter.setFilterFieldName(field_name);
        // 设置过滤的数值范围
        pfilter.setFilterLimits(limit_min, limit_max);
        // 使用setNegative() 方法设置是否保留过滤后的点
        pfilter.setNegative(negative_);
        // 执行过滤，将结果存储到cloud_filtered中
        pfilter.filter(*cloud_filtered);
        // 发射信号，传递过滤后的点云数据和函数执行时间
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::VoxelGrid(float lx, float ly, float lz)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        // 创建一个 VoxelGrid 滤波器对象
        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(cloud_);
        // 设置体素大小，
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative_);
        vfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ApproximateVoxelGrid(float lx, float ly, float lz)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(cloud_);
        avfilter.setLeafSize(lx, ly, lz);
        avfilter.filter(*cloud_filtered);

        if (negative_)
        {
            // pcl::ExtractIndices 是 PCL（Point Cloud Library）中的一个类，用于从点云中提取指定索引的点。
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            // avfilter.getRemovedIndices() 返回的是 ApproximateVoxelGrid 滤波器移除的点的索引
            // setIndices 设置了提取器要使用的索引，告诉 pcl::ExtractIndices 从原始点云中提取哪些点。
            extract.setIndices(avfilter.getRemovedIndices());
            // filter 方法执行过滤操作。它会根据提供的索引列表（avfilter.getRemovedIndices() 返回的索引）来提取出被移除的点。
            extract.filter(*cloud_filtered);
        }

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::StatisticalOutlierRemoval(int nr_k, double stddev_mult)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::StatisticalOutlierRemoval<PointXYZRGBN > sfilter;
        sfilter.setInputCloud(cloud_);
        // 设置邻近点数量
        sfilter.setMeanK(nr_k);
        // 设置标准差的乘数阈值 stddev_mult，用于判断哪些点是离群点
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative_);
        sfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::RadiusOutlierRemoval(double radius, int min_pts)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        // 设置邻域搜索半径
        rfilter.setRadiusSearch(radius);
        // 设置搜索半径内最小点数
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative_);
        rfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ConditionalRemoval(ConditionBase::Ptr con)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(cloud_);
        bfilter.setCondition(con);
        bfilter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN > extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(bfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::GridMinimun(const float resolution)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::GridMinimum<PointXYZRGBN > gfilter(resolution);
        gfilter.setInputCloud(cloud_);
        gfilter.setResolution(resolution);
        gfilter.setNegative(negative_);
        gfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::LocalMaximum(float radius)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::LocalMaximum<PointXYZRGBN> lfilter;
        lfilter.setInputCloud(cloud_);
        lfilter.setRadius(radius);
        lfilter.setNegative(negative_);
        lfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ShadowPoints(float threshold)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> sfilter;
        sfilter.setInputCloud(cloud_);
        sfilter.setNormals(cloud_);
        sfilter.setThreshold(threshold);
        sfilter.setNegative(negative_);
        sfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }
}
