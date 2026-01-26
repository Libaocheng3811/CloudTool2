//
// Created by LBC on 2025/1/10.
//

#include "modules/registration.h"

#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_rejection.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_sample_consensus_2d.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <pcl/registration/correspondence_sorting.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_fpcs.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/transformation_estimation_3point.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/registration/transformation_estimation_symmetric_point_to_plane_lls.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/transformation_validation_euclidean.h>

namespace ct
{
    // 该函数的主要目的是通过反向投影（Back Projection）方法来计算源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceEstimationBackProjection(int k)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN >::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN >::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        // 创建对应关系容器
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        //检查取消
        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        // 创建反向投影对应关系估计对象，用于估计源点云和目标点云之间的对应关系
        pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cebp
                (new pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cebp->setInputTarget(target_pcl);
        cebp->setInputSource(source_pcl);
        cebp->setSearchMethodTarget(target_tree);
        cebp->setSearchMethodSource(source_tree);
        cebp->setSourceNormals(source_pcl);
        cebp->setTargetNormals(target_pcl);
        cebp->setKSearch(k);

        if (m_is_canceled) return;
        emit progress(40);

        // 调用cebp对象的determineCorrespondences()函数，估计源点云和目标点云之间的对应关系
        cebp->determineCorrespondences(*corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceEstimationResult(corr, time.toc(), cebp);
    }

    // 该函数的主要目的是通过法线射线（Normal Shooting）的方法来计算源点云和目标点云之间的对应关系
    void Registration::CorrespondenceEstimationNormalShooting(int k)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cens
                (new pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cens->setInputTarget(target_pcl);
        cens->setInputSource(source_pcl);
        cens->setSourceNormals(source_pcl);
        cens->setSearchMethodTarget(target_tree);
        cens->setSearchMethodSource(source_tree);
        cens->setKSearch(k);

        if (m_is_canceled) return;
        emit progress(40);

        cens->determineCorrespondences(*corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceEstimationResult(corr, time.toc(), cens);
    }

    // 通过有序投影（Organized Projection）的方法来计算源点云和目标点云之间的对应关系
    void Registration::CorrespondenceEstimationOrganizedProjection(float fx, float fy, float cx, float cy,
                                                                   const Eigen::Matrix4f &src_to_tgt_trans,
                                                                   float depth_threshold) {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceEstimationOrganizedProjection<PointXYZRGBN, PointXYZRGBN>::Ptr ceop
                (new pcl::registration::CorrespondenceEstimationOrganizedProjection<PointXYZRGBN, PointXYZRGBN>);
        ceop->setInputTarget(target_pcl);
        ceop->setInputSource(source_pcl);
        ceop->setSearchMethodTarget(target_tree);
        ceop->setSearchMethodSource(source_tree);
        ceop->setFocalLengths(fx, fy);
        ceop->setCameraCenters(cx, cy);
        ceop->setSourceTransformation(src_to_tgt_trans);
        ceop->setDepthThreshold(depth_threshold);

        if (m_is_canceled) return;
        emit progress(40);

        double max_distance = 0;
        ceop->determineCorrespondences(*corr, max_distance);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceEstimationResult(corr, time.toc(), ceop);

    }

    // 根据给定的对应关系（correspondence）计算源点云和目标点云之间的对齐质量评分
    double Registration::DataContainer(const pcl::Correspondence &corr, bool from_normals)
    {
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::DataContainer<PointXYZRGBN, PointXYZRGBN> dc;
        dc.setInputTarget(target_pcl);
        dc.setInputSource(source_pcl);
        dc.setTargetNormals(target_pcl);
        dc.setSearchMethodTarget(target_tree);
        dc.setInputNormals(source_pcl);
        if (from_normals)
            return dc.getCorrespondenceScoreFromNormals(corr);
        else
            return dc.getCorrespondenceScore(corr);
    }

    // 主要功能是基于给定的最大距离阈值来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorDistance(float distance)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorDistance);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(corr_);
        cj->setMaximumDistance(distance);

        if (m_is_canceled) return;
        emit progress(50);

        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 目的是通过中位数距离过滤方法来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorMedianDistance(double factor)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorMedianDistance);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(corr_);
        cj->setMedianFactor(factor);

        if (m_is_canceled) return;
        emit progress(50);

        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要目的是通过一对一对应关系拒绝器（One-to-One Correspondence Rejector）来过滤源点云和目标点云之间的对应关系
    void Registration::CorrespondenceRejectorOneToOne()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cj(new pcl::registration::CorrespondenceRejectorOneToOne);

        if (m_is_canceled) return;
        emit progress(10);

        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(20);

        // 执行过滤操作
        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过有序边界对应关系拒绝器（CorrespondenceRejectionOrganizedBoundary）来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectionOrganizedBoundary(int val)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectionOrganizedBoundary cj;

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        cj.setInputTarget<PointXYZRGBN>(target_pcl);
        cj.setInputSource<PointXYZRGBN>(source_pcl);
        cj.setNumberOfBoundaryNaNs(val);
        cj.setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(40);

        cj.getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), std::make_shared<pcl::registration::CorrespondenceRejectionOrganizedBoundary>(cj));
    }

    // 主要功能是通过多边形点对过滤方法来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorPoly(int cardinality, float similarity_threshold, int iterations)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        cj->setInputTarget(target_pcl);
        cj->setInputSource(source_pcl);

        if (m_is_canceled) return;
        emit progress(10);

        cj->setCardinality(cardinality);
        cj->setSimilarityThreshold(similarity_threshold);
        cj->setIterations(iterations);
        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(50);

        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过样本一致性拒绝器（CorrespondenceRejectionSampleConsensus）来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorSampleConsensus(double threshold, int max_iterations, bool refine)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        cj->setInputTarget(target_pcl);
        cj->setInputSource(source_pcl);

        if (m_is_canceled) return;
        emit progress(10);

        cj->setInlierThreshold(threshold);
        cj->setMaximumIterations(max_iterations);
        cj->setRefineModel(refine);
        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(50);

        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要目的是通过表面法线一致性来过滤点云之间的对应关系。
    void Registration::CorrespondenceRejectorSurfaceNormal(double threshold)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();

        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr cj(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setThreshold(threshold);
        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(50);

        cj->getRemainingCorrespondences(*corr_, *corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过修剪对应关系的方式来过滤点云之间的对应关系
    void Registration::CorrespondenceRejectorTrimmed(float ratio, int min_corre)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorTrimmed);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        // cj->setSourceNormals(source_cloud_);
        // cj->setTargetNormals(target_cloud_);
        cj->setOverlapRatio(ratio);
        cj->setMinCorrespondences(min_corre);
        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(30);

        cj->getRemainingCorrespondences(*corr_,*corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 通过可变修剪（Variable Trimmed）方法过滤点云之间的对应关系。
    void Registration::CorrespondenceRejectorVarTrimmed(double min_ratio, double max_ratio)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorVarTrimmed);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        // cj->setSourceNormals(source_cloud_);
        // cj->setTargetNormals(target_cloud_);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setMinRatio(min_ratio);
        cj->setMaxRatio(max_ratio);
        cj->setInputCorrespondences(corr_);

        if (m_is_canceled) return;
        emit progress(40);

        cj->getRemainingCorrespondences(*corr_,*corr);

        if (m_is_canceled) return;
        emit progress(100);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 用GICP（广义迭代最近点）算法将源点云对齐到目标点云。
    void Registration::GeneralizedIterativeClosestPoint(int k, int max, double tra_tolerance,
                                                        double rol_tolerance, bool use_recip_corre)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::GeneralizedIterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(30);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        if (m_is_canceled) return;
        emit progress(50);

        reg.setCorrespondenceRandomness(k);
        reg.setMaximumOptimizerIterations(max);
        reg.setTranslationGradientTolerance(tra_tolerance);
        reg.setRotationGradientTolerance(rol_tolerance);
        reg.setUseReciprocalCorrespondences(use_recip_corre);
        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    // 通过FPCS算法将源点云对齐到目标点云。
    void Registration::FPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                                            float score_threshold, int nr_samples,
                                            float max_norm_diff, int max_runtime)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSourceNormals(source_pcl);
        reg.setTargetNormals(target_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(30);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setDelta(delta, normalize);
        reg.setApproxOverlap(approx_overlap);
        reg.setScoreThreshold(score_threshold);
        reg.setNumberOfSamples(nr_samples);
        reg.setMaxNormalDifference(max_norm_diff);
        reg.setMaxComputationTime(max_runtime);
        reg.setNumberOfThreads(14);

        if (m_is_canceled) return;
        emit progress(40);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::KFPCSInitialAlignment(float delta, bool normalize, float approx_overlap, float score_threshold,
                                             int nr_samples, float max_norm_diff, int max_runtime,
                                             float upper_trl_boundary, float lower_trl_boundary, float lambda)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSourceNormals(source_pcl);
        reg.setTargetNormals(target_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(30);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setDelta(delta, normalize);
        reg.setApproxOverlap(approx_overlap);
        reg.setScoreThreshold(score_threshold);
        reg.setNumberOfSamples(nr_samples);
        reg.setMaxNormalDifference(max_norm_diff);
        reg.setMaxComputationTime(max_runtime);

        reg.setUpperTranslationThreshold(upper_trl_boundary);
        reg.setLowerTranslationThreshold(lower_trl_boundary);
        reg.setLambda(lambda);

        if (m_is_canceled) return;
        emit progress(40);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPoint(bool use_recip_corre)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(20);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        if (m_is_canceled) return;
        emit progress(30);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPointWithNormals(bool use_recip_corre, bool use_symmetric_objective,
                                                        bool enforce_same_direction_normals)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(20);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setUseReciprocalCorrespondences(use_recip_corre);
        reg.setUseSymmetricObjective(use_symmetric_objective);
        reg.setEnforceSameDirectionNormals(enforce_same_direction_normals);

        if (m_is_canceled) return;
        emit progress(30);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPointNonLinear(bool use_recip_corre)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(20);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        if (m_is_canceled) return;
        emit progress(40);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::NormalDistributionsTransform(float resolution,
                                                    double step_size,
                                                    double outlier_ratio)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto target_pcl = target_cloud_->toPCL_XYZRGBN();
        auto source_pcl = source_cloud_->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);

        if (m_is_canceled) return;
        emit progress(20);

        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setResolution(resolution);
        reg.setStepSize(step_size);
        reg.setOulierRatio(outlier_ratio);

        if (m_is_canceled) return;
        emit progress(40);

        reg.align(*ail_pcl);

        if (m_is_canceled) return;
        emit progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::TransformationEstimation2D()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation2D<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimation3Point()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation3Point<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationDualQuaternion()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationDualQuaternion<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationLM()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationLM<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlane()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN, PointXYZRGBN>te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneLLS()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneLLSWeighted()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLSWeighted< PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneWeighted()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationSVD()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSVD<PointXYZRGBN, PointXYZRGBN> te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationSymmetricPointToPlaneLLS(bool enforce_same_direction_normals)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS< PointXYZRGBN, PointXYZRGBN>te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.setEnforceSameDirectionNormals(enforce_same_direction_normals);
        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationValidationEuclidean()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationValidationEuclidean<PointXYZRGBN, PointXYZRGBN>te;

        if (m_is_canceled) return;
        emit progress(10);

        auto source_pcl = source_cloud_->toPCL_XYZRGBN();
        auto target_pcl = target_cloud_->toPCL_XYZRGBN();

        te.validateTransformation(source_pcl, target_pcl, matrix);

        if (m_is_canceled) return;
        emit progress(100);

        emit transformationEstimationResult(matrix, time.toc());
    }
} // namespace ct