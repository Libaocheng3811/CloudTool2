#include "base/cloud.h"
#include "base/common.h"

#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <random>
#include <omp.h>

namespace ct
{
    std::vector<float> Cloud::s_jet_lut;

// 初始化颜色查找表 (LUT)
    void Cloud::initColorTable()
    {
        // 如果表已经初始化过（非空），直接返回，避免重复计算
        if (!s_jet_lut.empty()) return;

        // 定义表的大小，1024 足够平滑且占用内存极小 (4KB)
        int table_size = 1024;
        s_jet_lut.resize(table_size);

        for (int i = 0; i < table_size; ++i)
        {
            // 计算当前索引对应的归一化数值 v (0.0 ~ 1.0)
            float v = (float)i / (float)(table_size - 1);

            uint8_t r = 0, g = 0, b = 0;

            // Jet 颜色映射核心逻辑 (4段式)
            // 这里的 255 是颜色通道的最大值

            if (v < 0.25f)
            {
                // [0.00 ~ 0.25]: 蓝色 -> 青色 (Blue -> Cyan)
                // R: 0
                // G: 0 -> 255 (线性增加)
                // B: 255
                r = 0;
                g = (uint8_t)(255.0f * (4.0f * v));
                b = 255;
            }
            else if (v < 0.5f)
            {
                // [0.25 ~ 0.50]: 青色 -> 绿色 (Cyan -> Green)
                // R: 0
                // G: 255
                // B: 255 -> 0 (线性减少)
                // 公式说明: (v - 0.25) 是该段内的偏移量
                r = 0;
                g = 255;
                b = (uint8_t)(255.0f * (1.0f - 4.0f * (v - 0.25f)));
            }
            else if (v < 0.75f)
            {
                // [0.50 ~ 0.75]: 绿色 -> 黄色 (Green -> Yellow)
                // R: 0 -> 255 (线性增加)
                // G: 255
                // B: 0
                r = (uint8_t)(255.0f * (4.0f * (v - 0.5f)));
                g = 255;
                b = 0;
            }
            else
            {
                // [0.75 ~ 1.00]: 黄色 -> 红色 (Yellow -> Red)
                // R: 255
                // G: 255 -> 0 (线性减少)
                // B: 0
                r = 255;
                g = (uint8_t)(255.0f * (1.0f - 4.0f * (v - 0.75f)));
                b = 0;
            }

            std::uint32_t rgb_packed = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);

            // 使用 reinterpret_cast 将整数的位模式直接视为 float
            // 注意：这不是数值转换，而是内存重解释
            s_jet_lut[i] = *reinterpret_cast<float*>(&rgb_packed);
        }
    }

    // 设置点云颜色
    void Cloud::setCloudColor(const RGB &rgb)
    {
        if (!m_has_backup) backupColors();

        // 将rgb三个单独的颜色分量转换为一个32位整数
        std::uint32_t rgb_ =
                ((std::uint32_t) rgb.r << 16 | (std::uint32_t) rgb.g << 8 | (std::uint32_t) rgb.b);
        for (auto &i: points) i.rgb = *reinterpret_cast<float *>(&rgb_);

        m_color_modified = true;
    }

    // 设置点云颜色
    void Cloud::setCloudColor(const QString &axis)
    {
        if (!m_has_backup) backupColors();

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
        m_color_modified = true;
    }

    // 点云缩放
    void Cloud::scale(double x, double y, double z, bool origin)
    {

    }

    // 更新点云,包围盒、分辨率、
    void Cloud::update(bool box_flag, bool type_flag, bool resolution_flag)
    {
        // 空点云处理
        if (points.empty()) {
            m_type = CLOUD_TYPE_XYZ;
            m_resolution = 0.0f;
            m_box = {0,0,0, Eigen::Affine3f::Identity(), Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
            return;
        }

        if (type_flag)
        {
            // 判断是否有法线
            bool has_normals_data = false;
            size_t check_count = std::min(size_t(100), size());
            for (size_t i = 0; i < check_count; ++i)
            {
                if (points[i].normal_x != 0.0f || points[i].normal_y != 0.0f || points[i].normal_z != 0.0f)
                {
                    has_normals_data = true;
                    break;
                }
            }

            if (has_normals_data)
            {
                if (m_has_rgb) m_type = CLOUD_TYPE_XYZRGBN; // 有颜色+有法线
                else m_type = CLOUD_TYPE_XYZN; // 无颜色 + 有法线
            }
            else{
                if (m_has_rgb) m_type = CLOUD_TYPE_XYZRGB; // 有颜色+无法线
                else m_type = CLOUD_TYPE_XYZ; // 无颜色 + 无法线
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
                    m_resolution += sqrt(sqr_distances[1]);
                    // 有效点数量+1
                    ++n_points;
                }
            }
            if (n_points != 0) m_resolution /= n_points;
        }

        if (m_preview_cloud){
            m_preview_cloud = nullptr;
            if (size() > 30000000) generatePreview();
        }
    }

    void Cloud::generatePreview(int target_points) {
        // 如果点数本身就很少，不需要预览点云，直接用本身
        if (size() <= target_points){
            m_preview_cloud = nullptr;
            return;
        }

        // 如果已经生成过且大小差不多，就不重复计算
        if (m_preview_cloud && m_preview_cloud->size() > 0) return;

        m_preview_cloud.reset(new Cloud);

        // 步长采样，速度快，适合交互预览
        int step = size() / target_points;
        if (step < 1) step = 1;

        m_preview_cloud->reserve(target_points + 100);

        m_preview_cloud->setId(m_id + "_preview");
        m_preview_cloud->setInfo(m_info);
        m_preview_cloud->setHasRGB(m_has_rgb);

        for (size_t i = 0; i < size(); i += step){
            m_preview_cloud->push_back(points[i]);
        }
    }

    void Cloud::addScalarField(const QString &name, const std::vector<float> &values)
    {
        // 数据校验
        if (values.size() != size())
        {
            return;
        }
        m_scalar_fields[name] = values;
    }

    void Cloud::backupColors()
    {
        if (m_has_backup) return;

        m_original_colors.resize(size());
        for (size_t i = 0; i < size(); ++i)
        {
            m_original_colors[i] = *reinterpret_cast<std::uint32_t *>(&points[i].rgb);
        }
        m_has_backup = true;
        m_color_modified = false;
    }

    std::uint32_t Cloud::getPointColorForSave(size_t index) const
    {
        // 如果有备份数据，则使用备份数据
        if(m_has_backup && !m_original_colors.empty()){
            return m_original_colors[index];
        }
        // 没有备份数据，说明颜色数据没有被修改
        return *reinterpret_cast<const std::uint32_t *>(&points[index].rgb);
    }

    void Cloud::restoreColors()
    {
        if (!m_has_backup || !m_color_modified) return;

        for (std::size_t i = 0; i < size(); ++i)
        {
            points[i].rgb = *reinterpret_cast<float *>(&m_original_colors[i]);
        }
        m_has_backup = false;
        m_color_modified = false;
    }

    void Cloud::updateColorByField(const QString& field_name)
    {
        if (!hasScalarField(field_name)) return;

        // 确保LUT已初始化
        if (s_jet_lut.empty()) initColorTable();

        if (!m_has_backup) backupColors();

        const std::vector<float>& data = m_scalar_fields[field_name];

        size_t num_points = points.size();
        if (data.size() != num_points) return;

        // 计算极值
        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;

#pragma omp parallel
        {
            float local_min = FLT_MAX;
            float local_max = -FLT_MAX;

        #pragma omp for
            for (int i = 0; i < static_cast<int>(num_points); ++i)
            {
                float val = data[i];
                if (val < local_min) local_min = val;
                if (val > local_max) local_max = val;
            }

        #pragma omp critical
            {
                if (local_min < min_v) min_v = local_min;
                if (local_max > max_v) max_v = local_max;
            }
        }

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;

        const float* pData = data.data();
        const float* pLut = s_jet_lut.data();
        size_t lut_max_idx = s_jet_lut.size() - 1;

        //并行赋值
#pragma omp parallel for
        for (int i = 0; i < num_points; ++i) {
            // 归一化
            float norm = (pData[i] - min_v) / range;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;

            int idx = (int)(norm * lut_max_idx);

            points[i].rgb = pLut[idx];
        }
        m_color_modified = true;
    }

    void Cloud::removeInvalidPoints()
    {
        if (is_dense) return;

        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*this, *this, indices);

        // 这里的 indices 是 *保留下来* 的点的索引
        // 我们需要根据这些索引重建 scalar_fields

        if (indices.size() == m_original_colors.size()) return; // 没有点被删除

        if (!m_original_colors.empty()) {
            std::vector<std::uint32_t> new_colors;
            new_colors.reserve(indices.size());
            for (int idx : indices) new_colors.push_back(m_original_colors[idx]);
            m_original_colors = std::move(new_colors);
        }

        for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
            std::vector<float>& old_data = it.value();
            std::vector<float> new_data;
            new_data.reserve(indices.size());
            for (int idx : indices) new_data.push_back(old_data[idx]);
            it.value() = std::move(new_data);
        }
    }

    void Cloud::copyFrom(const ct::Cloud &other) {
        this->m_box = other.m_box;
        this->m_id = other.m_id;
        this->m_box_rgb = other.m_box_rgb;
        this->m_normals_rgb = other.m_normals_rgb;
        this->m_type = other.m_type;
        this->m_info = other.m_info;
        this->m_point_size = other.m_point_size;
        this->m_opacity = other.m_opacity;
        this->m_resolution = other.m_resolution;
        this->m_min = other.m_min;
        this->m_max = other.m_max;

        this->m_scalar_fields = other.m_scalar_fields; // QMap 也是值语义，会深拷贝
        this->m_original_colors = other.m_original_colors; // std::vector 深拷贝
        this->m_has_backup = other.m_has_backup;
        this->m_has_rgb = other.m_has_rgb;
        this->m_color_modified = other.m_color_modified;
        this->m_global_shift = other.m_global_shift;
        this->m_preview_cloud = nullptr;
    }
}
