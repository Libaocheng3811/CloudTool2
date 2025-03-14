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
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN >::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN >::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        // 创建对应关系容器
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        // 创建反向投影对应关系估计对象，用于估计源点云和目标点云之间的对应关系
        pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cebp
                (new pcl::registration::CorrespondenceEstimationBackProjection<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cebp->setInputTarget(target_cloud_);
        cebp->setInputSource(source_cloud_);
        cebp->setSearchMethodTarget(target_tree);
        cebp->setSearchMethodSource(source_tree);
        cebp->setSourceNormals(source_cloud_);
        cebp->setTargetNormals(target_cloud_);
        cebp->setKSearch(k);
        // 调用cebp对象的determineCorrespondences()函数，估计源点云和目标点云之间的对应关系
        cebp->determineCorrespondences(*corr);

        emit correspondenceEstimationResult(corr, time.toc(), cebp);
    }

    // 该函数的主要目的是通过法线射线（Normal Shooting）的方法来计算源点云和目标点云之间的对应关系
    void Registration::CorrespondenceEstimationNormalShooting(int k)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>::Ptr cens
                (new pcl::registration::CorrespondenceEstimationNormalShooting<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>);
        cens->setInputTarget(target_cloud_);
        cens->setInputSource(source_cloud_);
        cens->setSourceNormals(source_cloud_);
        cens->setSearchMethodTarget(target_tree);
        cens->setSearchMethodSource(source_tree);
        cens->setKSearch(k);
        cens->determineCorrespondences(*corr);

        emit correspondenceEstimationResult(corr, time.toc(), cens);
    }

    // 通过有序投影（Organized Projection）的方法来计算源点云和目标点云之间的对应关系
    void Registration::CorrespondenceEstimationOrganizedProjection(float fx, float fy, float cx, float cy,
                                                                   const Eigen::Matrix4f &src_to_tgt_trans,
                                                                   float depth_threshold) {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceEstimationOrganizedProjection<PointXYZRGBN, PointXYZRGBN>::Ptr ceop
                (new pcl::registration::CorrespondenceEstimationOrganizedProjection<PointXYZRGBN, PointXYZRGBN>);
        ceop->setInputTarget(target_cloud_);
        ceop->setInputSource(source_cloud_);
        ceop->setSearchMethodTarget(target_tree);
        ceop->setSearchMethodSource(source_tree);
        ceop->setFocalLengths(fx, fy);
        ceop->setCameraCenters(cx, cy);
        ceop->setSourceTransformation(src_to_tgt_trans);
        ceop->setDepthThreshold(depth_threshold);
        double max_distance = 0;
        ceop->determineCorrespondences(*corr, max_distance);

        emit correspondenceEstimationResult(corr, time.toc(), ceop);

    }

    // 根据给定的对应关系（correspondence）计算源点云和目标点云之间的对齐质量评分
    double Registration::DataContainer(const pcl::Correspondence &corr, bool from_normals)
    {
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::registration::DataContainer<PointXYZRGBN, PointXYZRGBN> dc;
        dc.setInputTarget(target_cloud_);
        dc.setInputSource(source_cloud_);
        dc.setTargetNormals(target_cloud_);
        dc.setSearchMethodTarget(target_tree);
        dc.setInputNormals(source_cloud_);
        if (from_normals)
            return dc.getCorrespondenceScoreFromNormals(corr);
        else
            return dc.getCorrespondenceScore(corr);
    }

    // 主要功能是基于给定的最大距离阈值来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorDistance(float distance)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceRejectorDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorDistance);
        cj->setInputTarget<PointXYZRGBN>(target_cloud_);
        cj->setInputSource<PointXYZRGBN>(source_cloud_);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(corr_);
        cj->setMaximumDistance(distance);
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 目的是通过中位数距离过滤方法来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorMedianDistance(double factor)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceRejectorMedianDistance::Ptr cj(new pcl::registration::CorrespondenceRejectorMedianDistance);
        cj->setInputTarget<PointXYZRGBN>(target_cloud_);
        cj->setInputSource<PointXYZRGBN>(source_cloud_);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setInputCorrespondences(corr_);
        cj->setMedianFactor(factor);
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要目的是通过一对一对应关系拒绝器（One-to-One Correspondence Rejector）来过滤源点云和目标点云之间的对应关系
    void Registration::CorrespondenceRejectorOneToOne()
    {
        TicToc time;
        time.tic();
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorOneToOne::Ptr cj(new pcl::registration::CorrespondenceRejectorOneToOne);

        cj->setInputCorrespondences(corr_);
        // 执行过滤操作
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过有序边界对应关系拒绝器（CorrespondenceRejectionOrganizedBoundary）来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectionOrganizedBoundary(int val)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectionOrganizedBoundary cj;

        cj.setInputTarget<PointXYZRGBN>(target_cloud_);
        cj.setInputSource<PointXYZRGBN>(source_cloud_);

        cj.setNumberOfBoundaryNaNs(val);
        cj.setInputCorrespondences(corr_);
        cj.getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), std::make_shared<pcl::registration::CorrespondenceRejectionOrganizedBoundary>(cj));
    }

    // 主要功能是通过多边形点对过滤方法来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorPoly(int cardinality, float similarity_threshold, int iterations)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorPoly<PointXYZRGBN, PointXYZRGBN>);
        cj->setInputTarget(target_cloud_);
        cj->setInputSource(source_cloud_);
        cj->setCardinality(cardinality);
        cj->setSimilarityThreshold(similarity_threshold);
        cj->setIterations(iterations);
        cj->setInputCorrespondences(corr_);
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过样本一致性拒绝器（CorrespondenceRejectionSampleConsensus）来过滤源点云和目标点云之间的对应关系（correspondences）
    void Registration::CorrespondenceRejectorSampleConsensus(double threshold, int max_iterations, bool refine)
    {
        TicToc time;
        time.tic();

        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>::Ptr cj(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZRGBN>);
        cj->setInputTarget(target_cloud_);
        cj->setInputSource(source_cloud_);
        cj->setInlierThreshold(threshold);
        cj->setMaximumIterations(max_iterations);
        cj->setRefineModel(refine);
        cj->setInputCorrespondences(corr_);
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要目的是通过表面法线一致性来过滤点云之间的对应关系。
    void Registration::CorrespondenceRejectorSurfaceNormal(double threshold)
    {
        TicToc time;
        time.tic();

        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr cj(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
        cj->setInputTarget<PointXYZRGBN>(target_cloud_);
        cj->setInputSource<PointXYZRGBN>(source_cloud_);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setThreshold(threshold);
        cj->setInputCorrespondences(corr_);
        cj->getRemainingCorrespondences(*corr_, *corr);

        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 主要功能是通过修剪对应关系的方式来过滤点云之间的对应关系
    void Registration::CorrespondenceRejectorTrimmed(float ratio, int min_corre)
    {
        TicToc time;
        time.tic();
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorTrimmed);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);
        // cj->setSourceNormals(source_cloud_);
        // cj->setTargetNormals(target_cloud_);
        cj->setOverlapRatio(ratio);
        cj->setMinCorrespondences(min_corre);
        cj->setInputCorrespondences(corr_);
        cj->getRemainingCorrespondences(*corr_,*corr);
        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 通过可变修剪（Variable Trimmed）方法过滤点云之间的对应关系。
    void Registration::CorrespondenceRejectorVarTrimmed(double min_ratio, double max_ratio)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::CorrespondencesPtr corr(new pcl::Correspondences);

        pcl::registration::CorrespondenceRejectorVarTrimmed::Ptr cj(new pcl::registration::CorrespondenceRejectorVarTrimmed);
        cj->setInputTarget<PointXYZRGBN>(target_cloud_);
        cj->setInputSource<PointXYZRGBN>(source_cloud_);
        // cj->setSourceNormals(source_cloud_);
        // cj->setTargetNormals(target_cloud_);
        cj->setSearchMethodTarget<PointXYZRGBN>(target_tree);
        cj->setMinRatio(min_ratio);
        cj->setMaxRatio(max_ratio);
        cj->setInputCorrespondences(corr_);
        cj->getRemainingCorrespondences(*corr_,*corr);
        emit correspondenceRejectorResult(corr, time.toc(), cj);
    }

    // 用GICP（广义迭代最近点）算法将源点云对齐到目标点云。
    void Registration::GeneralizedIterativeClosestPoint(int k, int max, double tra_tolerance,
                                                        double rol_tolerance, bool use_recip_corre)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GeneralizedIterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setCorrespondenceRandomness(k);
        reg.setMaximumOptimizerIterations(max);
        reg.setTranslationGradientTolerance(tra_tolerance);
        reg.setRotationGradientTolerance(rol_tolerance);
        reg.setUseReciprocalCorrespondences(use_recip_corre);
        reg.align(*ail_cloud);

        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    // 通过FPCS算法将源点云对齐到目标点云。
    void Registration::FPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                                            float score_threshold, int nr_samples,
                                            float max_norm_diff, int max_runtime)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::registration::FPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN>reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSourceNormals(source_cloud_);
        reg.setTargetNormals(target_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
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

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::KFPCSInitialAlignment(float delta, bool normalize, float approx_overlap, float score_threshold,
                                             int nr_samples, float max_norm_diff, int max_runtime,
                                             float upper_trl_boundary, float lower_trl_boundary, float lambda)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::registration::KFPCSInitialAlignment<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSourceNormals(source_cloud_);
        reg.setTargetNormals(target_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
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

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPoint(bool use_recip_corre)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::IterativeClosestPoint<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPointWithNormals(bool use_recip_corre, bool use_symmetric_objective,
                                                        bool enforce_same_direction_normals)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::IterativeClosestPointWithNormals<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
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

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::IterativeClosestPointNonLinear(bool use_recip_corre)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::IterativeClosestPointNonLinear<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
        for (auto& cj : cr_map) reg.addCorrespondenceRejector(cj.second);
        reg.setMaximumIterations(nr_iterations_);
        reg.setRANSACIterations(ransac_iterations_);
        reg.setRANSACOutlierRejectionThreshold(inlier_threshold_);
        reg.setMaxCorrespondenceDistance(distance_threshold_);
        reg.setTransformationEpsilon(transformation_epsilon_);
        reg.setTransformationRotationEpsilon(transformation_rotation_epsilon_);
        reg.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

        reg.setUseReciprocalCorrespondences(use_recip_corre);

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::NormalDistributionsTransform(float resolution,
                                                    double step_size,
                                                    double outlier_ratio)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr ail_cloud(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr target_tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::search::KdTree<PointXYZRGBN>::Ptr source_tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::NormalDistributionsTransform<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputTarget(target_cloud_);
        reg.setInputSource(source_cloud_);
        reg.setSearchMethodTarget(target_tree);
        reg.setSearchMethodSource(source_tree);
        if(te_!=nullptr) reg.setTransformationEstimation(te_);
        if(ce_!=nullptr) reg.setCorrespondenceEstimation(ce_);
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

        reg.align(*ail_cloud);
        emit registrationResult(reg.hasConverged(), ail_cloud, reg.getFitnessScore(),
                                reg.getFinalTransformation().cast<float>(), time.toc());
    }

    void Registration::TransformationEstimation2D()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation2D<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimation3Point()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimation3Point<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationDualQuaternion()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationDualQuaternion<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationLM()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationLM<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlane()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlane<PointXYZRGBN, PointXYZRGBN>te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneLLS()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLS<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneLLSWeighted()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneLLSWeighted< PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationPointToPlaneWeighted()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationSVD()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSVD<PointXYZRGBN, PointXYZRGBN> te;
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationEstimationSymmetricPointToPlaneLLS(bool enforce_same_direction_normals)
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationEstimationSymmetricPointToPlaneLLS< PointXYZRGBN, PointXYZRGBN>te;
        te.setEnforceSameDirectionNormals(enforce_same_direction_normals);
        te.estimateRigidTransformation(*source_cloud_, *target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }

    void Registration::TransformationValidationEuclidean()
    {
        TicToc time;
        time.tic();
        Eigen::Matrix4f matrix;
        pcl::registration::TransformationValidationEuclidean<PointXYZRGBN, PointXYZRGBN>te;
        te.validateTransformation(source_cloud_, target_cloud_, matrix);
        emit transformationEstimationResult(matrix, time.toc());
    }
} // namespace ct