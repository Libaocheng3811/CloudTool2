//
// Created by LBC on 2026/1/4.
//

#include "modules/csffilter.h"

#include <pcl/filters/extract_indices.h>

namespace ct{

    void extractScalarField(const Cloud::Ptr& source, Cloud::Ptr& target, const std::vector<int>& indices){
        target->setHasRGB(source->hasRGB());

        // 遍历源点云自定义字段
        QStringList fields = source->getScalarFieldNames();
        for (const QString& name : fields){
            const std::vector<float>* src_data = source->getScalarField(name);
            if (!src_data) continue;

            std::vector<float> tgt_data;
            tgt_data.reserve(indices.size());

            for (int idx : indices){
                if (idx >= 0 && idx < src_data->size()){
                    tgt_data.push_back((*src_data)[idx]);
                }
                else{
                    tgt_data.push_back(0.0f); //异常填充
                }
            }
            target->addScalarField(name, tgt_data);
        }
        target->backupColors();
    }

    void CSFFilter::applyCSF(bool bSloopSmooth, float time_step, double class_threshold, double cloth_resolution,
                            int rigidness, int iterations) {
        if (!cloud_ || cloud_->empty()) return;

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

        // 配置参数
        CSF csf;
        csf.setPointCloud(csf_points);
        csf.params.bSloopSmooth = bSloopSmooth;
        csf.params.time_step = time_step;
        csf.params.class_threshold = class_threshold;
        csf.params.cloth_resolution = cloth_resolution;
        csf.params.rigidness = rigidness;
        csf.params.interations = iterations;

        // 执行滤波
        std::vector<int> groundIndexes, offGroundIndexes;
        csf.do_filtering(groundIndexes, offGroundIndexes, true);

        // 将结果转换PCL Cloud
        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        ground_indices->indices = groundIndexes;

        pcl::PointIndices::Ptr off_ground_indices(new pcl::PointIndices);
        off_ground_indices->indices = offGroundIndexes;

        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(cloud_);

        // 提取地面点
        Cloud::Ptr ground_cloud(new Cloud);
        ground_cloud->setId(cloud_->id() + "_ground");
        extract.setIndices(ground_indices);
        extract.setNegative(false);
        extract.filter(*ground_cloud);
        extractScalarField(cloud_, ground_cloud, groundIndexes);

        // 提取非地面点
        Cloud::Ptr off_ground_cloud(new Cloud);
        off_ground_cloud->setId(cloud_->id() + "_off_ground");
        extract.setIndices(off_ground_indices);
        extract.setNegative(false);
        extract.filter(*off_ground_cloud);
        extractScalarField(cloud_, off_ground_cloud, offGroundIndexes);

        emit filterResult(ground_cloud, off_ground_cloud, time.toc());
    }

} // namespace ct