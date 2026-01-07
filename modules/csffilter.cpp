//
// Created by LBC on 2026/1/4.
//

#include "modules/csffilter.h"
#include "utils.h"
#include <pcl/filters/extract_indices.h>

namespace ct{
    void CSFFilter::applyCSF(bool bSloopSmooth, float time_step, double class_threshold,
                             double cloth_resolution, int rigidness, int iterations) {
        if (!cloud_ || cloud_->empty()) return;
        m_is_canceled = false;

        TicToc time;
        time.tic();

        // 数据转换 PCL Point -> CSF Point
        std::vector<csf::Point> csf_points;
        csf_points.reserve(cloud_->size());
        for(const auto& p : cloud_->points){
            csf::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            csf_points.emplace_back(pt);
        }

        if (m_is_canceled) return;
        emit progress(10);

        // 配置参数
        CSF csf;
        csf.setPointCloud(csf_points);
        csf.params.bSloopSmooth = bSloopSmooth;
        csf.params.time_step = time_step;
        csf.params.class_threshold = class_threshold;
        csf.params.cloth_resolution = cloth_resolution;
        csf.params.rigidness = rigidness;
        csf.params.interations = iterations;

        if (m_is_canceled) return;

        // 执行滤波
        std::vector<int> groundIndexes, offGroundIndexes;
        csf.do_filtering(groundIndexes, offGroundIndexes, true);

        if (m_is_canceled) return;
        emit progress(60);

        // 将结果转换PCL Cloud
        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        ground_indices->indices = groundIndexes;

        pcl::PointIndices::Ptr off_ground_indices(new pcl::PointIndices);
        off_ground_indices->indices = offGroundIndexes;

        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(cloud_);

        if (m_is_canceled) return;

        // 提取地面点
        ct::Cloud::Ptr ground_cloud(new Cloud);
        ground_cloud->setId(cloud_->id() + "_ground");
        extract.setIndices(ground_indices);
        extract.setNegative(false);
        extract.filter(*ground_cloud);

        if (m_is_canceled) return;

        syncAllScalarFields(cloud_, ground_cloud, groundIndexes);

        if (m_is_canceled) return;
        emit progress(80);

        // 提取非地面点
        ct::Cloud::Ptr off_ground_cloud(new Cloud);
        off_ground_cloud->setId(cloud_->id() + "_off_ground");
        extract.setIndices(off_ground_indices);
        extract.setNegative(false);
        if (m_is_canceled) return;
        extract.filter(*off_ground_cloud);

        if (m_is_canceled) return;

        syncAllScalarFields(cloud_, off_ground_cloud, offGroundIndexes);

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(ground_cloud, off_ground_cloud, time.toc());
    }

} // namespace ct