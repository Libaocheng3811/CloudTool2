#include "base/cloud.h"
#include "base/common.h"

#include <pcl/search/kdtree.h>

namespace ct
{
    // 设置点云颜色
    void Cloud::setCloudColor(const RGB &rgb)
    {
        // 将rgb三个单独的颜色分量转换为一个32位整数
        std::uint32_t rgb_ =
                ((std::uint32_t) rgb.r << 16 | (std::uint32_t) rgb.g << 8 | (std::uint32_t) rgb.b);
        for (auto &i: points) i.rgb = *reinterpret_cast<float *>(&rgb_);
    }

    // 设置点云颜色
    void Cloud::setCloudColor(const QString &axis)
    {
        float max = -FLT_MAX, min = FLT_MAX;
        float fRed = 0.f, fGreen = 0.f, fBlue = 0.f;
        constexpr float range = 2.f / 3.f;
        constexpr uint8_t colorMax = std::numeric_limits<uint8_t>::max();
        if (axis == "x")
        {
            max = m_max.x;
            min = m_min.x;
            for (auto& i : points)
            {
                float hue = (max - i.x) / static_cast<float >(max - min);
                hue *= range;
                hue = range - hue;
                HSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
                i.r = static_cast<uint8_t >(fRed * colorMax);
                i.g = static_cast<uint8_t >(fGreen * colorMax);
                i.b = static_cast<uint8_t >(fBlue * colorMax);
            }
        }
        else if (axis == "y")
        {
            max = m_max.y;
            min = m_min.y;
            for (auto& i : points)
            {
                float hue = (max - i.y) / static_cast<float >(max - min);
                hue *= range;
                hue = range - hue;
                HSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
                i.r = static_cast<uint8_t>(fRed * colorMax);
                i.g = static_cast<uint8_t>(fGreen * colorMax);
                i.b = static_cast<uint8_t >(fBlue * colorMax);
            }
        }
        else if (axis == "z")
        {
            max = m_max.z;
            min = m_min.z;
            for (auto& i : points)
            {
                float hue = (max - i.z) / static_cast<float >(max - min);
                hue *= range;
                hue = range - hue;
                HSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
                i.r = static_cast<uint8_t >(fRed * colorMax);
                i.g = static_cast<uint8_t >(fGreen * colorMax);
                i.b = static_cast<uint8_t >(fBlue * colorMax);
            }
        }
    }

    // 点云缩放
    void Cloud::scale(double x, double y, double z, bool origin)
    {

    }

    // 更新点云,包围盒、分辨率、
    void Cloud::update(bool box_flag, bool type_flag, bool resolution_flag)
    {
        // 如果更新点云类型，根据点云中任意一点的法向量和颜色信息来确定点云的类型
        if (type_flag)
        {
            if (points[rand() % size()].normal_x == 0.0f && points[rand() % size()].normal_y == 0.0f &&
                points[rand() % size()].normal_z == 0.0f)
            {
                // 如果颜色分量也为0，说明是没有颜色和法线信息的点云
                if (points[rand() % size()].r == 0 && points[rand() % size()].g == 0 &&
                        points[rand() % size()].b == 0)
                    m_type = CLOUD_TYPE_XYZ;
                else
                    m_type = CLOUD_TYPE_XYZRGB;
            }
            // 如果有法线信息
            else
            {
                if (points[rand() % size()].r == 0 && points[rand() % size()].g == 0 &&
                    points[rand() % size()].b == 0)
                    m_type = CLOUD_TYPE_XYZN;
                else
                    m_type = CLOUD_TYPE_XYARGBN;
            }
        }

        // 如果要更新包围盒
        if (box_flag)
        {
            pcl::getMinMax3D(*this, m_min, m_max);
            Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
            // 计算包围盒的宽度、高度、深度
            Eigen::Vector3f whd = m_max.getVector3fMap() - m_min.getVector3fMap();
            // 获取包围盒当前位姿
            Eigen::Affine3f affine = m_box.pose;
            float roll, pitch, yaw;
            pcl::getEulerAngles(affine, roll, pitch, yaw);
            affine = pcl::getTransformation(center[0], center[1], center[2], roll, pitch, yaw);
            m_box = {whd(0), whd(1), whd(2), affine, center, Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
        }
        if (resolution_flag)
        {
            int n_points = 0, nres, num = size() < 1000 ? size() : 1000;
            std::vector<int> indices(2);
            std::vector<float> sqr_distances(2);
            // 创建一个KdTree对象
            pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);
            tree->setInputCloud(this->makeShared());
            for (std::size_t i = 0; i < num; i++)
            {
                int index = rand() % size();
                if (!std::isfinite(points[index].x))
                    continue;
                nres = tree->nearestKSearch(index, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    // 分辨率计算方法的核心是估计点云中点之间的距离
                    m_resolution += sqrt(sqr_distances[1]);
                    // 有效点数量+1
                    ++n_points;
                }
            }
            if (n_points != 0) m_resolution /= n_points;
        }
    }

}
