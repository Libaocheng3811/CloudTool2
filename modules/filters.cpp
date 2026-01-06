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

        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::PassThrough<PointXYZRGBN> pfilter;
        pfilter.setInputCloud(cloud_);
        pfilter.setFilterFieldName(field_name);
        pfilter.setFilterLimits(limit_min, limit_max);
        pfilter.setNegative(negative_);
        pfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        // 创建一个 VoxelGrid 滤波器对象
        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(cloud_);
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative_);

        if (m_is_canceled) return;
        emit progress(20);

        vfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(cloud_);
        avfilter.setLeafSize(lx, ly, lz);
        avfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(avfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }

        if (m_is_canceled) return;

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::StatisticalOutlierRemoval<PointXYZRGBN > sfilter;
        sfilter.setInputCloud(cloud_);
        // 设置邻近点数量
        sfilter.setMeanK(nr_k);
        // 设置标准差的乘数阈值 stddev_mult，用于判断哪些点是离群点
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative_);
        sfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setRadiusSearch(radius);
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative_);
        rfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(cloud_);
        bfilter.setCondition(con);
        bfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN > extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(bfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }

        if (m_is_canceled) return;

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::GridMinimum<PointXYZRGBN > gfilter(resolution);
        gfilter.setInputCloud(cloud_);
        gfilter.setResolution(resolution);
        gfilter.setNegative(negative_);
        gfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::LocalMaximum<PointXYZRGBN> lfilter;
        lfilter.setInputCloud(cloud_);
        lfilter.setRadius(radius);
        lfilter.setNegative(negative_);
        lfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

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
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        if (m_is_canceled) return;
        emit progress(10);

        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> sfilter;
        sfilter.setInputCloud(cloud_);
        sfilter.setNormals(cloud_);
        sfilter.setThreshold(threshold);
        sfilter.setNegative(negative_);
        sfilter.filter(*cloud_filtered);

        if (m_is_canceled) return;
        emit progress(80);

        cloud_filtered->setId(cloud_->id());
        cloud_filtered->setHasRGB(cloud_->hasRGB());
        cloud_filtered->backupColors();

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(cloud_filtered, time.toc());
    }
}
