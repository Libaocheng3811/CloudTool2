//
// Created by LBC on 2026/1/26.
//

#ifndef CLOUDTOOL2_CLOUDTYPE_H
#define CLOUDTOOL2_CLOUDTYPE_H

#define _USE_MATH_DEFINES

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace ct{
    typedef pcl::PointXYZ PointXYZ;
    typedef pcl::PointXYZRGB PointXYZRGB;
    typedef pcl::PointXYZRGBNormal PointXYZRGBN;
    typedef pcl::Normal PointNormal;
    typedef pcl::Indices Indices;
    typedef pcl::console::TicToc TicToc;

    // 包围盒
    struct Box
    {
        double width = 0.0;
        double height = 0.0;
        double depth = 0.0;

        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();
        Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    };

    struct RGB
    {
        RGB() = default;
        RGB(uint8_t r_, uint8_t g_, uint8_t b_) :r(r_), g(g_), b(b_) {}
        double rf() const {return (double )r / 255; }
        double gf() const {return (double )g / 255; }
        double bf() const {return (double )b / 255; }
        uint8_t r = 255;
        uint8_t g = 255;
        uint8_t b = 255;
    };

    // 压缩法线（球面坐标编码，2 bytes）
    struct CompressedNormal
    {
        uint16_t data = 0;

        // 从 3D 向量编码
        void set(const Eigen::Vector3f& n)
        {
            float len = n.norm();
            if (len < 1e-6f) {
                data = 0;
                return;
            }

            Eigen::Vector3f normalized = n / len;

            // 计算 phi (极角): [0, π]
            float phi = std::acos(std::clamp(normalized.z(), -1.0f, 1.0f));

            // 计算 theta (方位角): [0, 2π]
            float theta = std::atan2(normalized.y(), normalized.x());
            if (theta < 0) theta += 2.0f * M_PI;

            // 编码: theta(9 bits) | phi(7 bits)
            uint16_t theta_bits = static_cast<uint16_t>(theta / (2.0f * M_PI) * 511.0f);
            uint16_t phi_bits = static_cast<uint16_t>(phi / M_PI * 127.0f);

            data = (theta_bits << 7) | phi_bits;
        }

        // 解码为 3D 向量
        Eigen::Vector3f get() const
        {
            uint16_t theta_bits = (data >> 7) & 0x1FF;
            uint16_t phi_bits = data & 0x7F;

            float theta = theta_bits / 511.0f * 2.0f * M_PI;
            float phi = phi_bits / 127.0f * M_PI;

            float sin_phi = std::sin(phi);

            return Eigen::Vector3f(
                    sin_phi * std::cos(theta),
                    sin_phi * std::sin(theta),
                    std::cos(phi)
            );
        }

        bool isZero() const { return data == 0; }
    };

    // define color
    namespace Color
    {
        const RGB White = {255, 255, 255};
        const RGB Black = {0, 0, 0};
        const RGB Red = {255, 0, 0};
        const RGB Green = { 0,  255,0 };
        const RGB Blue = { 0,  0,  255 };
        const RGB Yellow = { 255,255,0 };
        const RGB Cyan = { 0,255,255 };
        const RGB Purple = { 255,0,255 };
    }
} // namespace ct
#endif //CLOUDTOOL2_CLOUDTYPE_H
