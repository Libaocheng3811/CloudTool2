//
// Created by LBC on 2024/11/13.
//

#include "modules//features.h"

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
        // 定义一个点云最大点和最小点,存储坐标的最大和最小值
        PointXYZRGBN min, max;
        // 使用getMinMax3D 函数来计算点云中坐标最大和最小值，即找到点云中最大和最小的x,y,z值
        pcl::getMinMax3D(*cloud, min, max);
        // 使用一个三维向量来存储计算出的点云中心
        // getVector3fMap()将点的坐标转换为一个三维向量类型,返回一个 Eigen::Vector3f 的引用或映射
        Eigen::Vector3f cloud_center =
                0.5f * (min.getVector3fMap() + max.getVector3fMap());
        // 计算包围盒的宽高深
        Eigen::Vector3f whd;
        whd = max.getVector3fMap() - min.getVector3fMap();
        // 生成仿射变换矩阵，包括平移和旋转
        // 这里AABB包围盒不需要旋转，生成的仿射变换仅包含平移信息
        Eigen::Affine3f affine = pcl::getTransformation(
                cloud_center[0], cloud_center[1], cloud_center[2], 0, 0, 0);
        // 返回一个Box对象。
        // Eigen::Quaternionf(Eigen::Matrix3f::Identity())：这部分创建了一个单位四元数。
        // 单位四元数通常用于表示旋转。在这里使用单位四元数表明包围盒的没有进行任何旋转，保持为初始状态。
        return { whd(0), whd(1), whd(2), affine, cloud_center,
                 Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
    }

    Box Features::boundingBoxOBB(const Cloud::Ptr& cloud)
    {
        // 质心
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cloud, pcaCentroid);
        // 协方差
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
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
        Cloud::Ptr transformedCloud(new Cloud);
        transformPointCloud(*cloud, *transformedCloud, transform);
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
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        // 将t表示的仿射变换转换为标准的矩阵形式
        transform = t.matrix();
        Eigen::Matrix4f transform_inv = Eigen::Matrix4f::Identity();
        // 计算transform矩阵的逆矩阵
        transform_inv = transform.inverse();
        Cloud::Ptr transformedCloud(new Cloud);
        transformPointCloud(*cloud, *transformedCloud, transform);
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
        TicToc time;
        time.tic();
        // 括号内的 new FeatureType 表达式创建了一个新的 FeatureType 对象，并返回一个指向该对象的指针。
        // 这个指针随后被用来初始化智能指针 feature。
        FeatureType::Ptr feature(new FeatureType);
        // 为 feature 对象中的 pfh 成员分配了一个新的 PFHFeature 对象，这个对象将用于存储计算得到的PFH特征。
        feature->pfh.reset(new PFHFeature);
        // 创建了一个Kd树搜索对象 tree，用于在点云中快速搜索最近邻点。
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        // 定义了一个 pfh 对象，它是 pcl::PFHEstimation 类的实例，用于计算PFH特征。模板参数指定了输入点云和输出特征的类型，以及特征描述子的长度（125维）。
        pcl::PFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PFHSignature125> pfh;
        // 设置了PFH特征计算所需的搜索方法（即Kd树）
        pfh.setSearchMethod(tree);
        // 设置了搜索表面，这里假设 surface_ 是一个包含点云数据的成员变量
        pfh.setSearchSurface(surface_);
        // 设置输入点云
        pfh.setInputCloud(cloud_);
        // 设置输入法线
        pfh.setInputNormals(cloud_);
        // 设置了K近邻搜索的参数 k_，即考虑每个点的K个最近邻点来计算特征。
        pfh.setKSearch(k_);
        // 设置了半径搜索的参数 radius_，即在给定半径内搜索邻居点。
        pfh.setRadiusSearch(radius_);
        // 调用 compute 方法计算PFH特征，并将结果存储在 feature->pfh 中。
        // *feature->pfh表示获取 feature->pfh 智能指针所指向的 PFHFeature 对象的引用。
        pfh.compute(*feature->pfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::FPFHEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->fpfh.reset(new FPFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::FPFHEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::FPFHSignature33> fpfh;
        fpfh.setSearchMethod(tree);
        fpfh.setInputCloud(cloud_);
        fpfh.setInputNormals(cloud_);
        fpfh.setSearchSurface(surface_);
        fpfh.setKSearch(k_);
        fpfh.setRadiusSearch(radius_);
        fpfh.setNumberOfThreads(12);
        fpfh.compute(*feature->fpfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::VFHEstimation(const Eigen::Vector3f &dir)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::VFHEstimation<PointXYZRGBN , PointXYZRGBN , pcl::VFHSignature308> vfh;
        vfh.setSearchMethod(tree);
        vfh.setInputCloud(cloud_);
        vfh.setInputNormals(cloud_);
        vfh.setSearchSurface(surface_);
        vfh.setKSearch(k_);
        vfh.setRadiusSearch(radius_);
        vfh.setViewPoint(dir[0], dir[1], dir[2]);
        vfh.compute(*feature->vfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ESFEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->esf.reset(new ESFFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        pcl::ESFEstimation<PointXYZRGBN, pcl::ESFSignature640> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.compute(*feature->esf);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDEstimation(const Eigen::Vector3f &dir, int shgs, int shs, int interp)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasd.reset(new GASDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        pcl::GASDEstimation<PointXYZRGBN, pcl::GASDSignature512> est;
        est.setSearchMethod(tree);
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewDirection(dir);
        est.setShapeHalfGridSize(shgs);
        est.setShapeHistsSize(shs);
        est.setShapeHistsInterpMethod(pcl::HistogramInterpolationMethod(interp));
        est.compute(*feature->gasd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GASDColorEstimation(const Eigen::Vector3f &dir, int shgs, int shs, int interp, int chgs, int chs,
                                       int cinterp) {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->gasdc.reset(new GASDCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GASDColorEstimation<PointXYZRGBN , pcl::GASDSignature984> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
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
        est.compute(*feature->gasdc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::RSDEstimation(int nr_subdiv, double plane_radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->rsd.reset(new RSDFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::RSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::PrincipalRadiiRSD> est;
        est.setSearchMethod(tree);
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNrSubdivisions(nr_subdiv);
        est.setPlaneRadius(plane_radius);
        est.compute(*feature->rsd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::GRSDEstimation()
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->grsd.reset(new GRSDFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::GRSDEstimation<PointXYZRGBN, PointXYZRGBN, pcl::GRSDSignature21> est;
        est.setInputCloud(cloud_);
        est.setInputNormals(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.compute(*feature->grsd);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CRHEstimation(const Eigen::Vector3f &dir)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->crh.reset(new CRHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::CRHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::Histogram<90>> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.compute(*feature->crh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::CVFHEstimation(const Eigen::Vector3f &dir, float radius_normals, float d1, float d2, float d3,
                                  int min, bool normalize) {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->vfh.reset(new VFHFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::CVFHEstimation<PointXYZRGBN, PointXYZRGBN, pcl::VFHSignature308> est;
        est.setInputCloud(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setInputNormals(cloud_);
        est.setRadiusSearch(radius_);
        est.setViewPoint(dir[0], dir[1], dir[2]);
        est.setRadiusNormals(radius_normals);
        est.setClusterTolerance(d1);
        est.setEPSAngleThreshold(d2);
        est.setCurvatureThreshold(d3);
        est.setMinPoints(min);
        est.setNormalizeBins(normalize);
        est.compute(*feature->vfh);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::ShapeContext3DEstimation(double min_radius, double radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->sc3d.reset(new SC3DFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::ShapeContext3DEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ShapeContext1980> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(radius);
        est.compute(*feature->sc3d);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTEstimation(const ReferenceFrame::Ptr &lrf, float radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shot.reset(new SHOTFeature );
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::SHOTEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT352> shot;
        shot.setInputCloud(cloud_);
        shot.setInputNormals(cloud_);
        shot.setSearchMethod(tree);
        shot.setSearchSurface(surface_);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);
        shot.compute(*feature->shot);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTColorEstimation(const ReferenceFrame::Ptr &lrf, float radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->shotc.reset(new SHOTCFeature);
        pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);

        pcl::SHOTColorEstimationOMP<PointXYZRGBN, PointXYZRGBN, pcl::SHOT1344> shot;
        shot.setSearchMethod(tree);
        shot.setSearchSurface(surface_);
        shot.setInputCloud(cloud_);
        shot.setInputNormals(cloud_);
        shot.setKSearch(k_);
        shot.setRadiusSearch(radius_);
        shot.setNumberOfThreads(12);
        shot.setLRFRadius(radius);
        shot.setInputReferenceFrames(lrf);
        shot.compute(*feature->shotc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::UniqueShapeContext(const ReferenceFrame::Ptr &lrf, double min_radius,
                                      double pt_radius, double loc_radius)
    {
        TicToc time;
        time.tic();
        FeatureType::Ptr feature(new FeatureType);
        feature->usc.reset(new USCFeature);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::UniqueShapeContext<PointXYZRGBN, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> est;
        est.setSearchMethod(tree);
        est.setSearchSurface(surface_);
        est.setInputCloud(cloud_);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setInputReferenceFrames(lrf);
        est.setMinimalRadius(min_radius);
        est.setPointDensityRadius(pt_radius);
        est.setLocalRadius(loc_radius);
        est.compute(*feature->usc);
        emit featureResult(cloud_->id(), feature, time.toc());
    }

    void Features::BOARDLocalReferenceFrameEstimation(float radius, bool find_holes, float margin_thresh, int size,
                                                      float prob_thresh, float steep_thresh)
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::BOARDLocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setInputNormals(cloud_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setFindHoles(find_holes);
        est.setMarginThresh(margin_thresh);
        est.setCheckMarginArraySize(size);
        est.setHoleSizeProbThresh(prob_thresh);
        est.setSteepThresh(steep_thresh);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::FLARELocalReferenceFrameEstimation(float radius, float margin_thresh, int min_neighbors_for_normal_axis,
                                                      int min_neighbors_for_tangent_axis)
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::FLARELocalReferenceFrameEstimation<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame> est;
        est.setInputCloud(cloud_);
        est.setInputNormals(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setTangentRadius(radius);
        est.setMarginThresh(margin_thresh);
        est.setMinNeighboursForNormalAxis(min_neighbors_for_normal_axis);
        est.setMinNeighboursForTangentAxis(min_neighbors_for_tangent_axis);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

    void Features::SHOTLocalReferenceFrameEstimation()
    {
        TicToc time;
        time.tic();
        ReferenceFrame::Ptr feature(new ReferenceFrame);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::SHOTLocalReferenceFrameEstimationOMP<PointXYZRGBN, pcl::ReferenceFrame>
                est;
        est.setInputCloud(cloud_);
        est.setSearchSurface(surface_);
        est.setSearchMethod(tree);
        est.setKSearch(k_);
        est.setRadiusSearch(radius_);
        est.setNumberOfThreads(12);
        est.compute(*feature);
        emit lrfResult(cloud_->id(), feature, time.toc());
    }

} // namespace ct