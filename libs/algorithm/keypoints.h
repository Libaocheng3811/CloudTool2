//
// Created by LBC on 2025/1/9.
//

#ifndef MODULES_KEYPOINTS_H
#define MODULES_KEYPOINTS_H

#include "core/cloud.h"
#include "core/exports.h"

#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

namespace ct
{
    typedef pcl::PointXYZI          PointXYZI;
    typedef pcl::PointXYZRGB        PointXYZRGB;
    typedef pcl::PointWithScale     PointWithScale;
    typedef pcl::RangeImage         RangeImage;

    class Keypoints
    {
    };
} // namespace ct

#endif //MODULES_KEYPOINTS_H
