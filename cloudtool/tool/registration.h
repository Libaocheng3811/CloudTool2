//
// Created by LBC on 2025/1/10.
//

#ifndef CLOUDTOOL2_REGISTRATION_H
#define CLOUDTOOL2_REGISTRATION_H

#include "widgets/customdock.h"
#include "core/common.h"
#include "modules/registration.h"
#include "cloudtool/tool/descriptor.h"

#include <QThread>


QT_BEGIN_NAMESPACE
namespace Ui {
    class Registration;
}
QT_END_NAMESPACE

class Registration : public ct::CustomDock {
Q_OBJECT

public:
    explicit Registration(QWidget *parent = nullptr);


     ~Registration();

    // setDescriptor函数的主要功能是允许Registration类设置并管理一个Descriptor对象的引用
    void setDescriptor(Descriptor *descriptor) {
        m_descriptor = descriptor;
        // 当descriptor对象被销毁时，lambda函数将被调用，将m_descriptor成员变量设为nullptr
        if (descriptor) connect(descriptor, &Descriptor::destroyed, [=] { m_descriptor = nullptr; });
    }

    void setTarget();

    void setSource();

    void setCorrespondenceEstimation();

    void addCorrespondenceRejector();

    void removeCorrespondenceRejector();

    void setTransformationEstimation();

    void preview();

    void apply();

    void add();

    void reset();

signals:

    void CorrespondenceEstimationBackProjection(int k);

    void CorrespondenceEstimationNormalShooting(int k);

    void CorrespondenceRejectorDistance(float distance);

    void CorrespondenceRejectorMedianDistance(double factor);

    void CorrespondenceRejectorOneToOne();

    void CorrespondenceRejectionOrganizedBoundary(int val);

    void CorrespondenceRejectorPoly(int cardinality, float similarity_threshold, int iterations);

    void CorrespondenceRejectorSampleConsensus(double thershold, int max_iterations, bool refine);

    void CorrespondenceRejectorSurfaceNormal(double thershold);

    void CorrespondenceRejectorTrimmed(float ratio, int min_corre);

    void CorrespondenceRejectorVarTrimmed(double min_ratio, double max_ratio);

    void TransformationEstimation2D();

    void TransformationEstimation3Point();

    void TransformationEstimationDualQuaternion();

    void TransformationEstimationLM();

    void TransformationEstimationPointToPlane();

    void TransformationEstimationPointToPlaneLLS();

    void TransformationEstimationSVD();

    void TransformationEstimationSymmetricPointToPlaneLLS(bool enforce_same_direction_normals);

    void IterativeClosestPoint(bool use_recip_corre);

    void IterativeClosestPointWithNormals(bool use_recip_corre, bool use_symmetric_objective,
                                          bool enforce_same_direction_normals);

    void IterativeClosestPointNonLinear(bool use_recip_corre);

    void GeneralizedIterativeClosestPoint(int k, int max, double tra_tolerance, double rol_tolerance,
                                          bool use_recip_corre);

    void NormalDistributionsTransform(float resolution, double step_size, double outlier_ratio);

    void FPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                              float score_threshold, int nr_samples,
                              float max_norm_diff, int max_runtime);

    void KFPCSInitialAlignment(float delta, bool normalize, float approx_overlap,
                               float score_threshold, int nr_samples,
                               float max_norm_diff, int max_runtime,
                               float upper_trl_boundary, float lower_trl_boundary,
                               float lambda);

public slots:

    void correspondenceEstimationResult(const ct::CorrespondencesPtr& corr, float time, const ct::CorreEst::Ptr& ce = nullptr);

    void correspondenceRejectorResult(const ct::CorrespondencesPtr& corr, float time, const ct::CorreRej::Ptr& cj = nullptr);

    void transformationEstimationResult(const Eigen::Matrix4f& trans, float time, const ct::TransEst::Ptr& te = nullptr);

    void registrationResult(bool success, const ct::Cloud::Ptr& ail_cloud, double score, const Eigen::Matrix4f& matrix, float time);

private:
    Ui::Registration *ui;
    QThread m_thread;
    Descriptor *m_descriptor;
    ct::Registration *m_reg;
    ct::Cloud::Ptr m_target_cloud;
    ct::Cloud::Ptr m_source_cloud;
    ct::CorrespondencesPtr m_corr;
    ct::CorreEst::Ptr m_ce;
    ct::TransEst::Ptr m_te;
    std::map<int, ct::CorreRej::Ptr> m_cr_map;
    std::map<QString, ct::Cloud::Ptr> m_reg_map;
};


#endif //CLOUDTOOL2_REGISTRATION_H
