//
// Created by LBC on 2026/1/19.
//

#include "distancecalculator.h"

#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>
#include <omp.h>
#include <cmath>

namespace ct{

    static void computeNearest(const ct::Cloud::Ptr &ref, const ct::Cloud::Ptr &comp,
                               std::vector<float> &dists,
                               std::atomic<bool>* cancel,
                               const std::function<void(int)>& on_progress) {
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();
        pcl::search::KdTree<ct::PointXYZ> tree;
        tree.setInputCloud(refCloud);

        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for
        for (int i = 0; i < (int)n_points; ++i){
            if (cancel && cancel->load()) continue;

            std::vector<int> indices(1);
            std::vector<float> sqrt_dists(1);

            if (tree.nearestKSearch(compCloud->points[i], 1, indices, sqrt_dists) > 0){
                dists[i] = std::sqrt(sqrt_dists[0]);
            }
            else{
                dists[i] = std::numeric_limits<float>::quiet_NaN(); // 未找到对应点
            }

#pragma omp atomic
            progress_counter++;

            if (omp_get_thread_num() == 0 && progress_counter % (n_points / 50 + 1) == 0){
                if (on_progress) on_progress((progress_counter * 100) / n_points);
            }
        }
    }

    static void computeKnnMean(const ct::Cloud::Ptr &ref, const ct::Cloud::Ptr &comp, int k,
                               std::vector<float> &dists,
                               std::atomic<bool>* cancel,
                               const std::function<void(int)>& on_progress) {
        if (k < 1) k = 1;
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();

        pcl::search::KdTree<ct::PointXYZ> tree;
        tree.setInputCloud(refCloud);

        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for
        for (int i = 0; i < (int)n_points; ++i)
        {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices(k);
            std::vector<float> sqr_dists(k);

            int found = tree.nearestKSearch(compCloud->points[i], k, indices, sqr_dists);
            if (found > 0) {
                double sum = 0.0;
                for (float sq : sqr_dists) sum += std::sqrt(sq);
                dists[i] = static_cast<float>(sum / found);
            } else {
                dists[i] = std::numeric_limits<float>::quiet_NaN();
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (n_points / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / n_points);
            }
        }
    }

    static void computeRadiusMean(const ct::Cloud::Ptr &ref, const ct::Cloud::Ptr &comp, double r,
                                  std::vector<float> &dists,
                                  std::atomic<bool>* cancel,
                                  const std::function<void(int)>& on_progress) {
        auto refCloud = ref->toPCL_XYZ();
        auto compCloud = comp->toPCL_XYZ();

        pcl::search::KdTree<ct::PointXYZ> tree;
        tree.setInputCloud(refCloud);

        size_t n_points = comp->size();
        int progress_counter = 0;

#pragma omp parallel for
        for (int i = 0; i < (int)n_points; ++i)
        {
            if (cancel && cancel->load()) continue;

            std::vector<int> indices;
            std::vector<float> sqr_dists;

            int found = tree.radiusSearch(compCloud->points[i], r, indices, sqr_dists);
            if (found > 0) {
                double sum = 0.0;
                for (float sq : sqr_dists) sum += std::sqrt(sq);
                dists[i] = static_cast<float>(sum / found);
            } else {
                // 如果半径内没有点，距离定义为 NaN
                dists[i] = std::numeric_limits<float>::quiet_NaN();
            }

#pragma omp atomic
            progress_counter++;
            if (omp_get_thread_num() == 0 && progress_counter % (n_points / 50 + 1) == 0) {
                if (on_progress) on_progress((progress_counter * 100) / n_points);
            }
        }
    }

    DistanceResult DistanceCalculator::calculate(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                                  const DistanceParams& params,
                                                  std::atomic<bool>* cancel,
                                                  std::function<void(int)> on_progress) {
        if (cancel) cancel->store(false);

        if (!ref || !comp || ref->empty() || comp->empty()){
            return {{}, 0, false, "Invalid input clouds"};
        }

        pcl::console::TicToc timer;
        timer.tic();

        std::vector<float> distances(comp->size(), std::numeric_limits<float>::quiet_NaN());

        try{
            switch (params.method) {
                case DistanceParams::C2C_NEAREST:
                    computeNearest(ref, comp, distances, cancel, on_progress);
                    break;
                case DistanceParams::C2C_KNN_MEAN:
                    computeKnnMean(ref, comp, params.k_knn, distances, cancel, on_progress);
                    break;
                case DistanceParams::C2C_RADIUS_MEAN:
                    computeRadiusMean(ref, comp, params.radius, distances, cancel, on_progress);
                    break;
                default:
                    return {{}, 0, false, "Invalid distance method"};
            }
        }
        catch (const std::exception& e){
            return {{}, 0, false, std::string("Calculation error: ") + e.what()};
        }

        if (cancel && cancel->load()){
            return {{}, 0, false, "Calculation canceled"};
        }

        float duration = timer.toc() / 1000.0f;
        return {distances, duration, true, ""};
    }

} // namespace ct
