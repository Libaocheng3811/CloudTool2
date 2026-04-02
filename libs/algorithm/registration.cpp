//
// Created by LBC on 2025/1/10.
//

#include "libs/algorithm/registration.h"

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
    // 通过反向投影（Back Projection）方法来计算源点云和目标点云之间的对应关系
    Registration::CorrespondenceResult Registration::CorrespondenceEstimationBackProjection(const RegistrationContext& ctx, int k,
                                                                                            std::atomic<bool>* cancel,
                                                                                            std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cebp
                (new pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cebp->setInputTarget(target_pcl);
        cebp->setInputSource(source_pcl);
        cebp->setSearchMethodTarget(target_tree);
        cebp->setSearchMethodSource(source_tree);
        cebp->setSourceNormals(source_pcl);
        cebp->setTargetNormals(target_pcl);
        cebp->setKSearch(k);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        cebp->determineCorrespondences(*corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cebp, (float)time.toc()};
    }

    // 通过法线射线（Normal Shooting）的方法来计算源点云和目标点云之间的对应关系
    Registration::CorrespondenceResult Registration::CorrespondenceEstimationNormalShooting(const RegistrationContext& ctx, int k,
                                                                                           std::atomic<bool>* cancel,
                                                                                           std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cens
                (new pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cens->setInputTarget(target_pcl);
        cens->setInputSource(source_pcl);
        cens->setSourceNormals(source_pcl);
        cens->setSearchMethodTarget(target_tree);
        cens->setSearchMethodSource(source_tree);
        cens->setKSearch(k);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        cens->determineCorrespondences(*corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cens, (float)time.toc()};
    }

    // 通过有序投影（Organized Projection）的方法来计算源点云和目标点云之间的对应关系
    Registration::CorrespondenceResult Registration::CorrespondenceEstimationOrganizedProjection(
            const RegistrationContext& ctx, float fx, float fy, float cx, float cy,
            const Eigen::Matrix4f& src_to_tgt_trans, float depth_threshold,
            std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

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

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        double max_distance = 0;
        ceop->determineCorrespondences(*corr, max_distance);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, ceop, (float)time.toc()};
    }

    // 根据给定的对应关系（correspondence）计算源点云和目标点云之间的对齐质量评分
    double Registration::DataContainer(const RegistrationContext& ctx, const pcl::Correspondence& corr, bool from_normals)
    {
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

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

    // 基于给定的最大距离阈值来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorDistance(const RegistrationContext& ctx,
                                                                             const CorrespondencesPtr& input_corr, float distance,
                                                                             std::atomic<bool>* cancel,
                                                                             std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorDistance);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(input_corr);
        cj->setMaximumDistance(distance);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过中位数距离过滤方法来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorMedianDistance(const RegistrationContext& ctx,
                                                                                   const CorrespondencesPtr& input_corr, double factor,
                                                                                   std::atomic<bool>* cancel,
                                                                                   std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorMedianDistance);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(input_corr);
        cj->setMedianFactor(factor);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过一对一对应关系拒绝器来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorOneToOne(const RegistrationContext& ctx,
                                                                             const CorrespondencesPtr& input_corr,
                                                                             std::atomic<bool>* cancel,
                                                                             std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cj(new pcl::registration::CorrespondenceRejectorOneToOne);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(20);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过有序边界对应关系拒绝器来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectionOrganizedBoundary(const RegistrationContext& ctx,
                                                                                        const CorrespondencesPtr& input_corr, int val,
                                                                                        std::atomic<bool>* cancel,
                                                                                        std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectionOrganizedBoundary cj;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        cj.setInputTarget<PointXYZRGBN>(target_pcl);
        cj.setInputSource<PointXYZRGBN>(source_pcl);
        cj.setNumberOfBoundaryNaNs(val);
        cj.setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        cj.getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, std::make_shared<pcl::registration::CorrespondenceRejectionOrganizedBoundary>(cj), (float)time.toc()};
    }

    // 通过多边形点对过滤方法来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorPoly(const RegistrationContext& ctx,
                                                                          const CorrespondencesPtr& input_corr,
                                                                          int cardinality, float similarity_threshold, int iterations,
                                                                          std::atomic<bool>* cancel,
                                                                          std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        cj->setInputTarget(target_pcl);
        cj->setInputSource(source_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        cj->setCardinality(cardinality);
        cj->setSimilarityThreshold(similarity_threshold);
        cj->setIterations(iterations);
        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过样本一致性拒绝器来过滤源点云和目标点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorSampleConsensus(const RegistrationContext& ctx,
                                                                                     const CorrespondencesPtr& input_corr,
                                                                                     double threshold, int max_iterations, bool refine,
                                                                                     std::atomic<bool>* cancel,
                                                                                     std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        cj->setInputTarget(target_pcl);
        cj->setInputSource(source_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        cj->setInlierThreshold(threshold);
        cj->setMaximumIterations(max_iterations);
        cj->setRefineModel(refine);
        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过表面法线一致性来过滤点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorSurfaceNormal(const RegistrationContext& ctx,
                                                                                   const CorrespondencesPtr& input_corr, double threshold,
                                                                                   std::atomic<bool>* cancel,
                                                                                   std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();

        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr cj(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setThreshold(threshold);
        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过修剪对应关系的方式来过滤点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorTrimmed(const RegistrationContext& ctx,
                                                                             const CorrespondencesPtr& input_corr,
                                                                             float ratio, int min_corre,
                                                                             std::atomic<bool>* cancel,
                                                                             std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorTrimmed);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        cj->setOverlapRatio(ratio);
        cj->setMinCorrespondences(min_corre);
        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 通过可变修剪（Variable Trimmed）方法过滤点云之间的对应关系
    Registration::RejectorResult Registration::CorrespondenceRejectorVarTrimmed(const RegistrationContext& ctx,
                                                                               const CorrespondencesPtr& input_corr,
                                                                               double min_ratio, double max_ratio,
                                                                               std::atomic<bool>* cancel,
                                                                               std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorVarTrimmed);
        cj->setInputTarget<PointXYZRGBN>(target_pcl);
        cj->setInputSource<PointXYZRGBN>(source_pcl);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setMinRatio(min_ratio);
        cj->setMaxRatio(max_ratio);
        cj->setInputCorrespondences(input_corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        cj->getRemainingCorrespondences(*input_corr, *corr);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {corr, cj, (float)time.toc()};
    }

    // 用GICP（广义迭代最近点）算法将源点云对齐到目标点云
    Registration::RegistrationResult Registration::GeneralizedIterativeClosestPoint(const RegistrationContext& ctx,
                                                                                   int k, int max, double tra_tolerance,
                                                                                   double rol_tolerance, bool use_recip_corre,
                                                                                   std::atomic<bool>* cancel,
                                                                                   std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::GeneralizedIterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(50);

        reg.setCorrespondenceRandomness(k);
        reg.setMaximumOptimizerIterations(max);
        reg.setTranslationGradientTolerance(tra_tolerance);
        reg.setRotationGradientTolerance(rol_tolerance);
        reg.setUseReciprocalCorrespondences(use_recip_corre);
        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    // 通过FPCS算法将源点云对齐到目标点云
    Registration::RegistrationResult Registration::FPCSInitialAlignment(const RegistrationContext& ctx,
                                                                         float delta, bool normalize, float approx_overlap,
                                                                         float score_threshold, int nr_samples,
                                                                         float max_norm_diff, int max_runtime,
                                                                         std::atomic<bool>* cancel,
                                                                         std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSourceNormals(source_pcl);
        reg.setTargetNormals(target_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setDelta(delta, normalize);
        reg.setApproxOverlap(approx_overlap);
        reg.setScoreThreshold(score_threshold);
        reg.setNumberOfSamples(nr_samples);
        reg.setMaxNormalDifference(max_norm_diff);
        reg.setMaxComputationTime(max_runtime);
        reg.setNumberOfThreads(14);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::RegistrationResult Registration::KFPCSInitialAlignment(const RegistrationContext& ctx,
                                                                          float delta, bool normalize, float approx_overlap, float score_threshold,
                                                                          int nr_samples, float max_norm_diff, int max_runtime,
                                                                          float upper_trl_boundary, float lower_trl_boundary, float lambda,
                                                                          std::atomic<bool>* cancel,
                                                                          std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSourceNormals(source_pcl);
        reg.setTargetNormals(target_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setDelta(delta, normalize);
        reg.setApproxOverlap(approx_overlap);
        reg.setScoreThreshold(score_threshold);
        reg.setNumberOfSamples(nr_samples);
        reg.setMaxNormalDifference(max_norm_diff);
        reg.setMaxComputationTime(max_runtime);

        reg.setUpperTranslationThreshold(upper_trl_boundary);
        reg.setLowerTranslationThreshold(lower_trl_boundary);
        reg.setLambda(lambda);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::RegistrationResult Registration::IterativeClosestPoint(const RegistrationContext& ctx,
                                                                         bool use_recip_corre,
                                                                         std::atomic<bool>* cancel,
                                                                         std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(20);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::RegistrationResult Registration::IterativeClosestPointWithNormals(const RegistrationContext& ctx,
                                                                                    bool use_recip_corre, bool use_symmetric_objective,
                                                                                    bool enforce_same_direction_normals,
                                                                                    std::atomic<bool>* cancel,
                                                                                    std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(20);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setUseReciprocalCorrespondences(use_recip_corre);
        reg.setUseSymmetricObjective(use_symmetric_objective);
        reg.setEnforceSameDirectionNormals(enforce_same_direction_normals);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(30);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::RegistrationResult Registration::IterativeClosestPointNonLinear(const RegistrationContext& ctx,
                                                                                  bool use_recip_corre,
                                                                                  std::atomic<bool>* cancel,
                                                                                  std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(20);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::RegistrationResult Registration::NormalDistributionsTransform(const RegistrationContext& ctx,
                                                                                 float resolution,
                                                                                 double step_size,
                                                                                 double outlier_ratio,
                                                                                 std::atomic<bool>* cancel,
                                                                                 std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();
        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();

        pcl::PointCloud<PointXYZRGBN>::Ptr ail_pcl(new pcl::PointCloud<PointXYZRGBN>);
        pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_pcl);
        reg.setInputSource(source_pcl);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(ctx.transformation_estimation!=nullptr) reg.setTransformationEstimation(ctx.transformation_estimation);
        if(ctx.correspondence_estimation!=nullptr) reg.setCorrespondenceEstimation(ctx.correspondence_estimation);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(20);

        for (auto& cj : ctx.correspondence_rejectors) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(ctx.params.max_iterations);
        reg.setRANSACIterations(ctx.params.ransac_iterations);
        reg.setRANSACOutlierRejectionThreshold(ctx.params.inlier_threshold);
        reg.setMaxCorrespondenceDistance(ctx.params.distance_threshold);
        reg.setTransformationEpsilon(ctx.params.transformation_epsilon);
        reg.setTransformationRotationEpsilon(ctx.params.transformation_rotation_epsilon);
        reg.setEuclideanFitnessEpsilon(ctx.params.euclidean_fitness_epsilon);

        reg.setResolution(resolution);
        reg.setStepSize(step_size);
        reg.setOulierRatio(outlier_ratio);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(40);

        reg.align(*ail_pcl);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        Cloud::Ptr ail_cloud = Cloud::fromPCL_XYZRGBN(*ail_pcl);
        return {reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                reg.getFinalTransformation(), (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimation2D(const RegistrationContext& ctx,
                                                                                 const CorrespondencesPtr& input_corr,
                                                                                 std::atomic<bool>* cancel,
                                                                                 std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation2D<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimation3Point(const RegistrationContext& ctx,
                                                                                     const CorrespondencesPtr& input_corr,
                                                                                     std::atomic<bool>* cancel,
                                                                                     std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation3Point<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationDualQuaternion(const RegistrationContext& ctx,
                                                                                             const CorrespondencesPtr& input_corr,
                                                                                             std::atomic<bool>* cancel,
                                                                                             std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationDualQuaternion<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationLM(const RegistrationContext& ctx,
                                                                                 const CorrespondencesPtr& input_corr,
                                                                                 std::atomic<bool>* cancel,
                                                                                 std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationLM<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationPointToPlane(const RegistrationContext& ctx,
                                                                                            const CorrespondencesPtr& input_corr,
                                                                                            std::atomic<bool>* cancel,
                                                                                            std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationPointToPlaneLLS(const RegistrationContext& ctx,
                                                                                              const CorrespondencesPtr& input_corr,
                                                                                              std::atomic<bool>* cancel,
                                                                                              std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationPointToPlaneLLSWeighted(const RegistrationContext& ctx,
                                                                                                     const CorrespondencesPtr& input_corr,
                                                                                                     std::atomic<bool>* cancel,
                                                                                                     std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLSWeighted<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationPointToPlaneWeighted(const RegistrationContext& ctx,
                                                                                                 const CorrespondencesPtr& input_corr,
                                                                                                 std::atomic<bool>* cancel,
                                                                                                 std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationSVD(const RegistrationContext& ctx,
                                                                                 const CorrespondencesPtr& input_corr,
                                                                                 std::atomic<bool>* cancel,
                                                                                 std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSVD<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    Registration::TransformationResult Registration::TransformationEstimationSymmetricPointToPlaneLLS(
            const RegistrationContext& ctx, const CorrespondencesPtr& input_corr,
            bool enforce_same_direction_normals,
            std::atomic<bool>* cancel, std::function<void(int)> on_progress)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS<PointXYZRGBN, PointXYZRGBN> te;

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(10);

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.setEnforceSameDirectionNormals(enforce_same_direction_normals);
        te.estimateRigidTransformation(*source_pcl, *target_pcl, matrix);

        if (cancel && *cancel) return {};
        if (on_progress) on_progress(100);

        return {matrix, nullptr, (float)time.toc()};
    }

    void Registration::TransformationValidationEuclidean(const RegistrationContext& ctx,
                                                          const pcl::Correspondence& corr, bool from_normals)
    {
        pcl::registration::TransformationValidationEuclidean<PointXYZRGBN, PointXYZRGBN> te;

        auto source_pcl = ctx.source_cloud->toPCL_XYZRGBN();
        auto target_pcl = ctx.target_cloud->toPCL_XYZRGBN();

        te.validateTransformation(source_pcl, target_pcl, DataContainer(ctx, corr, from_normals));
    }
} // namespace ct
