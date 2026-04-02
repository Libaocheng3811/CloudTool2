//
// Created by LBC on 2026/1/6.
//

#ifndef CLOUDTOOL2_VEGFILTER_H
#define CLOUDTOOL2_VEGFILTER_H

#include "core/cloud.h"
#include "core/exports.h"

#include <functional>
#include <atomic>

namespace ct{
    enum class VegIndexType {
        ExG_ExR = 0,
        ExG,
        NGRDI,
        CIVE
    };

    struct VegResult {
        Cloud::Ptr veg_cloud;
        Cloud::Ptr non_veg_cloud;
        float time_ms = 0;
    };

    class CT_EXPORT VegetationFilter {
    public:
        static VegResult apply(const Cloud::Ptr& cloud,
                                int index_type, double threshold,
                                std::atomic<bool>* cancel = nullptr,
                                std::function<void(int)> on_progress = nullptr);
    };
}


#endif //CLOUDTOOL2_VEGFILTER_H
