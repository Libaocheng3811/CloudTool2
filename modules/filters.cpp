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
    // 辅助函数，同步属性和自定义字段
    void syncCloudProperties(const Cloud::Ptr& source, Cloud::Ptr& target){
        target->setId(source->id());
        target->setHasRGB(source->hasRGB());

        // TODO: 使用PCL的filter无法返回索引，就无法通过索引提取自定义字段信息，这里暂时丢弃自定义字段信息

        target->backupColors();
    }

    void Filters::PassThrough(const std::string& field_name, float limit_min, float limit_max)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        // 转换输入点云
        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::PassThrough<PointXYZRGBN> pfilter;
        pfilter.setInputCloud(pcl_cloud);
        pfilter.setFilterFieldName(field_name);
        pfilter.setFilterLimits(limit_min, limit_max);
        pfilter.setNegative(negative_);
        pfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        // 从 PCL 结果构造 Cloud
        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::VoxelGrid(float lx, float ly, float lz)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        // 创建一个 VoxelGrid 滤波器对象
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(pcl_cloud);
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative_);

        if (m_is_canceled) return;
        emit progress(20);

        vfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ApproximateVoxelGrid(float lx, float ly, float lz)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(pcl_cloud);
        avfilter.setLeafSize(lx, ly, lz);
        avfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        if (negative_)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(avfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (m_is_canceled) return;

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::StatisticalOutlierRemoval(int nr_k, double stddev_mult)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(pcl_cloud);
        sfilter.setMeanK(nr_k);
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative_);
        sfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::RadiusOutlierRemoval(double radius, int min_pts)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(pcl_cloud);
        rfilter.setRadiusSearch(radius);
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative_);
        rfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ConditionalRemoval(ConditionBase::Ptr con)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(pcl_cloud);
        bfilter.setCondition(con);
        bfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        if (negative_)
        {
            pcl::PointCloud<PointXYZRGBN>::Ptr pcl_neg_filtered(new pcl::PointCloud<PointXYZRGBN>);
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(bfilter.getRemovedIndices());
            extract.filter(*pcl_neg_filtered);
            pcl_filtered = pcl_neg_filtered;
        }

        if (m_is_canceled) return;

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::GridMinimun(const float resolution)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::GridMinimum<PointXYZRGBN> gfilter(resolution);
        gfilter.setInputCloud(pcl_cloud);
        gfilter.setResolution(resolution);
        gfilter.setNegative(negative_);
        gfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::LocalMaximum(float radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::LocalMaximum<PointXYZRGBN> lfilter;
        lfilter.setInputCloud(pcl_cloud);
        lfilter.setRadius(radius);
        lfilter.setNegative(negative_);
        lfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ShadowPoints(float threshold)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_filtered(new pcl::PointCloud<PointXYZRGBN>);
        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> sfilter;
        sfilter.setInputCloud(pcl_cloud);
        sfilter.setNormals(pcl_cloud);
        sfilter.setThreshold(threshold);
        sfilter.setNegative(negative_);
        sfilter.filter(*pcl_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        Cloud::Ptr cloud_filtered = Cloud::fromPCL_XYZRGBN(*pcl_filtered);
        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }
}
