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
        auto pclCloud = cloud_->toPCL_XYZ();
        std::vector<csf::Point> csf_points;
        csf_points.resize(pclCloud->size());  // 使用 resize 而不是 reserve

#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(pclCloud->size()); ++i) {
            const auto& p = pclCloud->points[i];
            csf_points[i].x = p.x;
            csf_points[i].y = p.y;
            csf_points[i].z = p.z;
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

        // 使用 fromPCL 方法从 PCL 点云构造 Cloud
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_ground(new pcl::PointCloud<PointXYZRGBN>);
        pcl::PointCloud<PointXYZRGBN>::Ptr pcl_off_ground(new pcl::PointCloud<PointXYZRGBN>);

        pcl_ground->resize(groundIndexes.size());
        pcl_off_ground->resize(offGroundIndexes.size());

        auto pcl_cloud_xyzrgbn = cloud_->toPCL_XYZRGBN();

#pragma omp parallel for
        for (size_t i = 0; i < groundIndexes.size(); ++i) {
            if (groundIndexes[i] >= 0 && groundIndexes[i] < pcl_cloud_xyzrgbn->size()) {
                pcl_ground->points[i] = pcl_cloud_xyzrgbn->points[groundIndexes[i]];
            }
        }

#pragma omp parallel for
        for (size_t i = 0; i < offGroundIndexes.size(); ++i) {
            if (offGroundIndexes[i] >= 0 && offGroundIndexes[i] < pcl_cloud_xyzrgbn->size()) {
                pcl_off_ground->points[i] = pcl_cloud_xyzrgbn->points[offGroundIndexes[i]];
            }
        }

        if (m_is_canceled) return;
        emit progress(80);

        ct::Cloud::Ptr ground_cloud = Cloud::fromPCL_XYZRGBN(*pcl_ground);
        ground_cloud->setId(cloud_->id() + "_ground");
        syncAllScalarFields(cloud_, ground_cloud, groundIndexes);

        if (m_is_canceled) return;
        emit progress(90);

        ct::Cloud::Ptr off_ground_cloud = Cloud::fromPCL_XYZRGBN(*pcl_off_ground);
        off_ground_cloud->setId(cloud_->id() + "_off_ground");
        syncAllScalarFields(cloud_, off_ground_cloud, offGroundIndexes);

        if (m_is_canceled) return;
        emit progress(100);

        emit filterResult(ground_cloud, off_ground_cloud, time.toc());
    }

} // namespace ct