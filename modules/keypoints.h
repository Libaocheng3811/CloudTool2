//
// Created by LBC on 2025/1/9.
//

#ifndef MODULES_KEYPOINTS_H
#define MODULES_KEYPOINTS_H

#include <QObject>

#include "core/cloud.h"

#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>

namespace ct
{
    typedef pcl::PointXYZI          PointXYZI;
    typedef pcl::PointXYZRGB        PointXYZRGB;
    typedef pcl::PointWithScale     PointWithScale;
    typedef pcl::RangeImage         RangeImage;

    class CT_EXPORT Keypoints : public QObject
    {
        Q_OBJECT

    public:
        explicit Keypoints(QObject* parent = nullptr) :
                QObject(parent), cloud_(nullptr), surface_(nullptr), k_(0), radius_(0)
        {}

    private:
        Cloud::Ptr cloud_;
        Cloud::Ptr surface_;
        int k_;
        double radius_;
    };
} // namespace ct


#endif //MODULES_KEYPOINTS_H
