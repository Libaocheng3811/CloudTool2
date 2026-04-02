//
// Created by LBC on 2026/1/19.
//

#ifndef CLOUDTOOL2_DISTANCECALCULATOR_H
#define CLOUDTOOL2_DISTANCECALCULATOR_H

#include <vector>
#include <functional>
#include <atomic>
#include <string>
#include "core/cloud.h"
#include "core/exports.h"
#include "field_types.h"

namespace ct {

    struct DistanceResult {
        std::vector<float> distances;
        float time_ms = 0;
        bool success = true;
        std::string error_msg;
    };

    class CT_EXPORT DistanceCalculator {
    public:
        static DistanceResult calculate(const Cloud::Ptr& ref, const Cloud::Ptr& comp,
                                         const DistanceParams& params,
                                         std::atomic<bool>* cancel = nullptr,
                                         std::function<void(int)> on_progress = nullptr);
    };
} // namespace ct

#endif //CLOUDTOOL2_DISTANCECALCULATOR_H
