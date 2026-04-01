//
// Created by LBC on 2024/11/13.
//

#include "modules/features.h"

#include <pcl/features/boundary.h>
#include <pcl/features/impl/3dsc.hpp>
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/crh.hpp>
#include <pcl/features/impl/cvfh.hpp>
#include <pcl/features/impl/don.hpp>
#include <pcl/features/impl/esf.hpp>
#include <pcl/features/impl/flare.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/gasd.hpp>
#include <pcl/features/impl/grsd.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/rsd.hpp>
#include <pcl/features/impl/shot.hpp>
#include <pcl/features/impl/shot_lrf.hpp>
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>
#include <pcl/features/impl/usc.hpp>
#include <pcl/features/impl/vfh.hpp>

namespace ct
{
    Box Features::boundingBoxAABB(const Cloud::Ptr& cloud)
    {
        auto pcl_cloud = cloud->toPCL_XYZRGBN();
        PointXYZRGBN min, max;
        pcl::getMinMax3D(*pcl_cloud, min, max);
        Eigen::Vector3f cloud_center =
                0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd;
        whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f affine = pcl::getTransformation(
                cloud_center[0], cloud_center[1], cloud_center[2], 0, 0, 0);
        return { whd(0), whd(1), whd(2), affine, cloud_center,
                 Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
    }

    Box Features::boundingBoxOBB(const Cloud::Ptr& cloud)
    {
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        // 质心
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*pcl_cloud, pcaCentroid);
        // 协方差
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*pcl_cloud, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors(); // feature vector
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
        eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        transform.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());
        transform_inv = transform.inverse();

        pcl::PointCloud<PointXYZRGBN>::Ptr transformedCloud(new pcl::PointCloud<PointXYZRGBN>);
        transformPointCloud(*pcl_cloud, *transformedCloud, transform);

        PointXYZRGBN min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (max.getVector3fMap() + min.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return {whd(0), whd(1), whd(2), transform_inv_aff, tcloud_center,
                Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0))};

    }

    Box Features::boundingBoxAdjust(const Cloud::Ptr& cloud, const Eigen::Affine3f &t)
    {
        auto pcl_cloud = cloud->toPCL_XYZRGBN();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        // 将t表示的仿射变换转换为标准的矩阵形式
        transform = t.matrix();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        // 计算transform矩阵的逆矩阵
        transform_inv = transform.inverse();

        pcl::PointCloud<PointXYZRGBN>::Ptr transformedCloud(new pcl::PointCloud<PointXYZRGBN>);
        transformPointCloud(*pcl_cloud, *transformedCloud, transform);

        PointXYZRGBN min, max;
        Eigen::Vector3f cloud_center, tcloud_center;
        pcl::getMinMax3D(*transformedCloud, min, max);
        cloud_center = 0.5f * (min.getVector3fMap() + max.getVector3fMap());
        Eigen::Vector3f whd = max.getVector3fMap() - min.getVector3fMap();
        Eigen::Affine3f transform_inv_aff(transform_inv);
        pcl::transformPoint(cloud_center, tcloud_center, transform_inv_aff);
        return {whd(0), whd(1), whd(2), transform_inv_aff, tcloud_center,
                Eigen::Quaternionf(transform_inv.block<3, 3>(0, 0)) };
    }

    void Features::PFHEstimation()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->pfh.reset(new PFHFeature);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PFHSignature125> pfh;
        pfh.setSearchMethod(tree);
        pfh.setSearchSurface(pcl_surface);
        pfh.setInputCloud(pcl_cloud);
        pfh.setInputNormals(pcl_cloud);
        pfh.setKSearch(k_);
        pfh.setRadiusSearch(radius_);

        if (m_is_canceled) return;
        emit progress(20);

        pfh.compute(*feature->pfh);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::FPFHEstimation()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->fpfh.reset(new FPFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::FPFHEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> fpfh;
        fpfh.setSearchMethod(tree);
        fpfh.setInputCloud(pcl_cloud);
        fpfh.setInputNormals(pcl_cloud);
        fpfh.setSearchSurface(pcl_surface);
        fpfh.setKSearch(k_);
        fpfh.setRadiusSearch(radius_);
        fpfh.setNumberOfThreads(12);

        if (m_is_canceled) return;
        emit progress(30);

        fpfh.compute(*feature->fpfh);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::VFHEstimation(const Eigen::Vector3f &dir)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::VFHEstimation<PointXYZRGBN , PointXYZRGBN , pcl::VFHSignature308> vfh;
        vfh.setSearchMethod(tree);
        vfh.setInputCloud(pcl_cloud);
        vfh.setInputNormals(pcl_cloud);
        vfh.setSearchSurface(pcl_surface);
        vfh.setKSearch(k_);
        vfh.setRadiusSearch(radius_);
        vfh.setViewPoint(dir[0], dir[1], dir[2]);

        if (m_is_canceled) return;
        emit progress(30);

        vfh.compute(*feature->vfh);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ESFEstimation()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->esf.reset(new ESFFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::ESFEstimation<PointXYZRGBN, pcl::ESFSignature640> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->esf);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDEstimation(const Eigen::Vector3f &dir, int shgs, int shs, int interp)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasd.reset(new GASDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::GASDEstimation<PointXYZRGBN, pcl::GASDSignature512> est;
        est.setSearchMethod(tree);
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->gasd);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDColorEstimation(const Eigen::Vector3f &dir, int shgs, int shs, int interp, int chgs, int chs,
                                       int cinterp) {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasdc.reset(new GASDCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::GASDColorEstimation<PointXYZRGBN , pcl::GASDSignature984> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.setColorHalfGridSize(chgs);
        est.setColorHistsSize(chs);
        est.setColorHistsInterpMethod(pcl::HistogramInterpolationMethod(cinterp));

        if (m_is_canceled) return;
        emit progress(40);

        est.compute(*feature->gasdc);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::RSDEstimation(int nr_subdiv, double plane_radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->rsd.reset(new RSDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::RSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PrincipalRadiiRSD> est;
        est.setSearchMethod(tree);
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setInputNormals(pcl_cloud);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNrSubdivisions(nr_subdiv);
        est.setPlaneRadius(plane_radius);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->rsd);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GRSDEstimation()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->grsd.reset(new GRSDFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::GRSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::GRSDSignature21> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->grsd);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CRHEstimation(const Eigen::Vector3f &dir)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->crh.reset(new CRHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::CRHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Histogram<90>> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->crh);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CVFHEstimation(const Eigen::Vector3f &dir, float radius_normals, float d1, float d2, float d3,
                                  int min, bool normalize) {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();

        pcl::CVFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::VFHSignature308> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setInputNormals(pcl_cloud);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.setRadiusNormals(radius_normals);
        est.setClusterTolerance(d1);
        est.setEPSAngleThreshold(d2);
        est.setCurvatureThreshold(d3);
        est.setMinPoints(min);
        est.setNormalizeBins(normalize);

        if (m_is_canceled) return;
        emit progress(40);

        est.compute(*feature->vfh);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ShapeContext3DEstimation(double min_radius, double radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->sc3d.reset(new SC3DFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::ShapeContext3DEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ShapeContext1980> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(radius);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->sc3d);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTEstimation(const ReferenceFrame::Ptr &lrf, float radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shot.reset(new SHOTFeature );
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::SHOTEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT352> shot;
        shot.setInputCloud(pcl_cloud);
        shot.setInputNormals(pcl_cloud);
        shot.setSearchMethod(tree);
        shot.setSearchSurface(pcl_surface);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);

        if (m_is_canceled) return;
        emit progress(30);

        shot.compute(*feature->shot);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTColorEstimation(const ReferenceFrame::Ptr &lrf, float radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shotc.reset(new SHOTCFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::SHOTColorEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT1344> shot;
        shot.setSearchMethod(tree);
        shot.setSearchSurface(pcl_surface);
        shot.setInputCloud(pcl_cloud);
        shot.setInputNormals(pcl_cloud);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);

        if (m_is_canceled) return;
        emit progress(40);

        shot.compute(*feature->shotc);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::UniqueShapeContext(const ReferenceFrame::Ptr &lrf, double min_radius,
                                      double pt_radius, double loc_radius)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->usc.reset(new USCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::UniqueShapeContext<PointXYZRGBN, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> est;
        est.setSearchMethod(tree);
        est.setSearchSurface(pcl_surface);
        est.setInputCloud(pcl_cloud);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setInputReferenceFrames(lrf);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(pt_radius);
        est.setLocalRadius(loc_radius);

        if (m_is_canceled) return;
        emit progress(30);

        est.compute(*feature->usc);

        if (m_is_canceled) return;
        emit progress(100);

        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::BOARDLocalReferenceFrameEstimation(float radius, bool find_holes, float margin_thresh, int size,
                                                      float prob_thresh, float steep_thresh)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::BOARDLocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setInputNormals(pcl_cloud);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setFindHoles(find_holes);
        est.setMarginThresh(margin_thresh);
        est.setCheckMarginArraySize(size);
        est.setHoleSizeProbThresh(prob_thresh);
        est.setSteepThresh(steep_thresh);

        if (m_is_canceled) return;
        emit progress(40);

        est.compute(*feature);

        if (m_is_canceled) return;
        emit progress(100);

        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::FLARELocalReferenceFrameEstimation(float radius, float margin_thresh, int min_neighbors_for_normal_axis,
                                                      int min_neighbors_for_tangent_axis)
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::FLARELocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setInputNormals(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setMarginThresh(margin_thresh);
        est.setMinNeighboursForNormalAxis(min_neighbors_for_normal_axis);
        est.setMinNeighboursForTangentAxis(min_neighbors_for_tangent_axis);

        if (m_is_canceled) return;
        emit progress(40);

        est.compute(*feature);

        if (m_is_canceled) return;
        emit progress(100);

        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTLocalReferenceFrameEstimation()
    {
        m_is_canceled = false;

        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        if (m_is_canceled) return;
        emit progress(10);

        auto pcl_cloud = cloud_->toPCL_XYZRGBN();
        auto pcl_surface = surface_ ? surface_->toPCL_XYZRGBN() : pcl_cloud;

        pcl::SHOTLocalReferenceFrameEstimationOMP<PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(pcl_cloud);
        est.setSearchSurface(pcl_surface);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNumberOfThreads(12);

        if (m_is_canceled) return;
        emit progress(40);

        est.compute(*feature);

        if (m_is_canceled) return;
        emit progress(100);

        emit lrfResult(cloud_->id(), feature, time.toc());
    }

} // namespace ct