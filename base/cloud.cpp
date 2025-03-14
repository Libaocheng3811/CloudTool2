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
        // FLT_MAX表示 float 类型所能表示的最大值的正数线性极限
        float max = -FLT_MAX, min = FLT_MAX;
        // 0.f表示单精度浮点数0
        float fRed = 0.f, fGreen = 0.f, fBlue = 0.f;
        // constexpr关键字，用于指示某个变量或函数的值可以在编译时求得
        // 当一个变量被声明为constexpr时，这意味着它的值在编译时就已经确定，并且不可更改。这样可以提高代码的性能
        constexpr float range = 2.f / 3.f;
        // 将 colorMax 声明为一个常量，且其值被设置为 uint8_t 类型的最大值 255
        // std::numeric_limits<T> 是一个模板类，提供了关于数值类型 T 的一些属性和特性。max()可获取类型可表示的最大值
        constexpr uint8_t colorMax = std::numeric_limits<uint8_t>::max();
        if (axis == "x")
        {
            max = m_max.x;
            min = m_min.x;
            for (auto& i : points)
            {
                // 计算当前点的色调值，并归一化
                float hue = (max - i.x) / static_cast<float >(max - min);
                // 调整色调值，是x坐标较大的点对应较低的色调值
                hue *= range;
                hue = range - hue;
                // 色调值转为RGB，饱和度和明度在这里都设置为1.f，表明该颜色是纯色，没有灰度（满饱和度）且亮度达到最大值。
                HSVtoRGB(hue, 1.f, 1.f, fRed, fGreen, fBlue);
                // 将转换后的 RGB 颜色分量存储到点 i 的 r、g 和 b 属性中。这里的 colorMax 为 255，确保颜色值在 0 到 255 的范围内
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
            // normal_x 是PCL中点云数据结构的一个成员变量，它代表点云中每个点的法线向量在 x 轴上的分量。
            // 如果任意一点在三个方向上的法线分量都为0，说明该点没有法向量信息
            if (points[rand() % size()].normal_x == 0.0f && points[rand() % size()].normal_y == 0.0f &&
                points[rand() % size()].normal_z == 0.0f)
            {
                // 如果颜色分量也为0，说明是没有颜色和法线信息的点云
                if (points[rand() % size()].r == 0 && points[rand() % size()].g == 0 &&
                        points[rand() % size()].b == 0)
                    m_type = CLOUD_TYPE_XYZ;
                // 如果颜色分量不为0，说明是有颜色但是没有法线信息的点云
                else
                    m_type = CLOUD_TYPE_XYZRGB;
            }
            // 如果有法线信息
            else
            {
                // 如果有颜色信息，就是有颜色和法线的点云，否则就是有法线没颜色的点云
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
            // pcl::getMinMax3D 是一个函数，用于计算点云数据的最小和最大边界框。
            // 这个函数会找出点云中所有点的最小和最大 x、y、z 坐标，这些坐标定义了一个轴对齐的边界框
            // 就是将最小的x,y,z存储到m_min中，最大的一致
            pcl::getMinMax3D(*this, m_min, m_max);
            // 计算包围盒的中心，通过最小点和最大点的平均值得到
            // getVector3fMap()是PointXYZRGBN 类的一个成员函数，返回一个指向点的 x、y、z 坐标的 Eigen::Vector3f 引用
            // 这个函数允许我们直接访问点的三维位置信息，而忽略其他信息（如颜色和法线）
            Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
            // 计算包围盒的宽度、高度、深度
            Eigen::Vector3f whd = m_max.getVector3fMap() - m_min.getVector3fMap();
            // 获取包围盒当前位姿
            Eigen::Affine3f affine = m_box.pose;
            float roll, pitch, yaw;
            // pcl::getEulerAngles用于从给定的仿射变换（Eigen::Affine3f 类型）中提取欧拉角
            // pitch俯仰角，yaw偏航角， roll滚转角
            pcl::getEulerAngles(affine, roll, pitch, yaw);
            // getTransformation函数生成一个三维仿射变换矩阵，前3个参数表示沿3个轴的平移量，后三个角度表示沿轴的旋转角度
            affine = pcl::getTransformation(center[0], center[1], center[2], roll, pitch, yaw);
            // 设置新的包围盒属性
            // Eigen::Quaternionf 是 Eigen 库中定义的一个类，用于表示四元数
            // Eigen::Matrix3f::Identity() 是 Eigen 库中的一个函数调用，它用于创建一个 3x3 的单精度浮点数（float 类型）单位矩阵（Identity Matrix）
            // []：这通常用于数组或标准库中的容器（如std::vector）来访问元素, 但Eigen库的向量类支持使用括号访问元素，
            // 该类实现了()运算符重载以方便地访问其元素。这也是Eigen库的一种编码风格，强调数学表达的直观性。
            m_box = {whd(0), whd(1), whd(2), affine, center, Eigen::Quaternionf(Eigen::Matrix3f::Identity())};
        }
        if (resolution_flag)
        {
            // 三元运算符，如果size<1000, num = size(), 否则num = 1000
            // size()是PointCloud类的成员函数，
            // pcl::PointCloud 类是一个模板类，可以处理不同类型的点类型，例如 pcl::PointXYZ、pcl::PointXYZRGB 等
            int n_points = 0, nres, num = size() < 1000 ? size() : 1000;
            std::vector<int> indices(2);
            std::vector<float> sqr_distances(2);
            // 创建一个KdTree对象
            pcl::search::KdTree<PointXYZRGBN >::Ptr tree(new pcl::search::KdTree<PointXYZRGBN >);
            tree->setInputCloud(this->makeShared());
            // 通过kd树对点云数据进行查询，以计算点云的平均分辨率
            // num表示在点云中要随机选择的点数量，size_t表示无符号整数类型
            for (std::size_t i = 0; i < num; i++)
            {
                // 随机选取一个点的索引
                int index = rand() % size();
                // 检查这个点的X坐标是否是有限的（即不是无穷大或 NaN）
                if (!std::isfinite(points[index].x))
                    continue;
                // 使用Kd树对象 tree 执行K-近邻搜索，查找与当前索引 index 最近的两个点（参数2表示查找2个点）,存储在 indices（最近邻的索引）和 sqr_distances（与这些点的距离平方）中
                // nearestKSearch 是 KdTree 类的一个成员函数，它找到给定点的最近 k 个邻居。返回找到的最邻近点的数量
                nres = tree->nearestKSearch(index, 2, indices, sqr_distances);
                // nearestKSearch 函数并不一定能保证总是返回2个邻近点，由于有限点数、点无法访问或点稀疏，无法提供足够的邻近点
                if (nres == 2)
                {
                    // 分辨率计算方法的核心是估计点云中点之间的距离
                    m_resolution += sqrt(sqr_distances[1]);
                    // 有效点数量+1
                    ++n_points;
                }
            }
            // 如果找到了一些有效点，计算平均分辨率，代表了点云中点之间的平均距离
            if (n_points != 0) m_resolution /= n_points;
        }
    }

}
