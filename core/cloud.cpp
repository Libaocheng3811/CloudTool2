#include "cloud.h"
#include "common.h"

#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter_indices.h>
#include <random>
#include <omp.h>
#include <QSet>

namespace ct
{
    std::vector<float> Cloud::s_jet_lut;

    // ===== 构造/析构 =====
    Cloud::Cloud()
        : m_xyz(new pcl::PointCloud<PointXYZ>),
          m_preview(nullptr),
          m_id("cloud"),
          m_box_rgb(Color::White),
          m_normals_rgb(Color::White)
    {
    }

    Cloud::~Cloud() = default;

    Cloud::Cloud(Cloud&& other) noexcept
    {
        m_xyz = std::move(other.m_xyz);
        m_colors = std::move(other.m_colors);
        m_normals = std::move(other.m_normals);
        m_scalar_fields = std::move(other.m_scalar_fields);

        m_cached_xyzrgb = std::move(other.m_cached_xyzrgb);
        m_cached_xyzrgbn = std::move(other.m_cached_xyzrgbn);
        m_render_cloud = std::move(other.m_render_cloud);
        m_preview = std::move(other.m_preview);

        m_id = std::move(other.m_id);
        m_info = std::move(other.m_info);
        m_box = other.m_box;
        m_box_rgb = other.m_box_rgb;
        m_normals_rgb = other.m_normals_rgb;
        m_point_size = other.m_point_size;
        m_opacity = other.m_opacity;
        m_resolution = other.m_resolution;
        m_min = other.m_min;
        m_max = other.m_max;
        m_global_shift = other.m_global_shift;
        m_has_rgb = other.m_has_rgb;
        m_has_normals = other.m_has_normals;
        m_type = std::move(other.m_type);
        m_original_colors = std::move(other.m_original_colors);
        m_has_backup = other.m_has_backup;
        m_color_modified = other.m_color_modified;

        m_cache_valid = false;
        m_render_cache_valid = false;

        // 重置 other
        other.m_has_rgb = false;
        other.m_has_normals = false;
        other.m_has_backup = false;
        other.m_color_modified = false;
    }

    Cloud& Cloud::operator=(Cloud&& other) noexcept
    {
        if (this != &other) {
            m_xyz = std::move(other.m_xyz);
            m_colors = std::move(other.m_colors);
            m_normals = std::move(other.m_normals);
            m_scalar_fields = std::move(other.m_scalar_fields);

            m_cached_xyzrgb = std::move(other.m_cached_xyzrgb);
            m_cached_xyzrgbn = std::move(other.m_cached_xyzrgbn);
            m_render_cloud = std::move(other.m_render_cloud);
            m_preview = std::move(other.m_preview);

            m_id = std::move(other.m_id);
            m_info = std::move(other.m_info);
            m_box = other.m_box;
            m_box_rgb = other.m_box_rgb;
            m_normals_rgb = other.m_normals_rgb;
            m_point_size = other.m_point_size;
            m_opacity = other.m_opacity;
            m_resolution = other.m_resolution;
            m_min = other.m_min;
            m_max = other.m_max;
            m_global_shift = other.m_global_shift;
            m_has_rgb = other.m_has_rgb;
            m_has_normals = other.m_has_normals;
            m_type = std::move(other.m_type);
            m_original_colors = std::move(other.m_original_colors);
            m_has_backup = other.m_has_backup;
            m_color_modified = other.m_color_modified;

            m_cache_valid = false;
            m_render_cache_valid = false;

            other.m_has_rgb = false;
            other.m_has_normals = false;
            other.m_has_backup = false;
            other.m_color_modified = false;
        }
        return *this;
    }

    // ===== 容量接口 =====
    size_t Cloud::size() const
    {
        return m_xyz->size();
    }

    bool Cloud::empty() const
    {
        return m_xyz->empty();
    }

    void Cloud::reserve(size_t n)
    {
        m_xyz->reserve(n);
        if (m_colors) m_colors->reserve(n);
        if (m_normals) m_normals->reserve(n);
    }

    void Cloud::resize(size_t n)
    {
        m_xyz->resize(n);
        if (m_colors) m_colors->resize(n, Color::White);
        if (m_normals) m_normals->resize(n);
        invalidateCache();
    }

    void Cloud::clear()
    {
        m_xyz->clear();
        m_colors.reset();
        m_normals.reset();
        m_scalar_fields.clear();
        m_preview.reset();
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_render_cloud.reset();
        m_has_rgb = false;
        m_has_normals = false;
        invalidateCache();
    }

    // ===== 点访问 =====
    Cloud::PointRef Cloud::operator[](size_t index)
    {
        if (index >= m_xyz->size()) {
            throw std::out_of_range("Cloud index out of range");
        }

        auto& pt = m_xyz->points[index];
        bool has_rgb = m_colors != nullptr && index < m_colors->size();
        bool has_normal = m_normals != nullptr && index < m_normals->size();

        return PointRef(
            pt.x, pt.y, pt.z,
            has_rgb, has_normal,
            has_rgb ? &(*m_colors)[index] : nullptr,
            has_normal ? &(*m_normals)[index] : nullptr
        );
    }

    Cloud::ConstPointRef Cloud::operator[](size_t index) const
    {
        if (index >= m_xyz->size()) {
            throw std::out_of_range("Cloud index out of range");
        }

        const auto& pt = m_xyz->points[index];
        bool has_rgb = m_colors != nullptr && index < m_colors->size();
        bool has_normal = m_normals != nullptr && index < m_normals->size();

        return ConstPointRef(
            pt.x, pt.y, pt.z,
            has_rgb, has_normal,
            has_rgb ? &(*m_colors)[index] : nullptr,
            has_normal ? &(*m_normals)[index] : nullptr
        );
    }

    Cloud::PointRef Cloud::at(size_t index)
    {
        return (*this)[index];
    }

    Cloud::ConstPointRef Cloud::at(size_t index) const
    {
        return (*this)[index];
    }

    // ===== 批量添加 =====
    void Cloud::addPoint(float x, float y, float z)
    {
        m_xyz->push_back(PointXYZ(x, y, z));
        if (m_colors) m_colors->push_back(Color::White);
        if (m_normals) m_normals->push_back(CompressedNormal());
        invalidateCache();
    }

    void Cloud::addPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
    {
        m_xyz->push_back(PointXYZ(x, y, z));
        if (m_colors) {
            m_colors->push_back(RGB(r, g, b));
        } else {
            enableColors();
            m_colors->resize(m_xyz->size() - 1, Color::White);
            m_colors->push_back(RGB(r, g, b));
        }
        if (m_normals) m_normals->push_back(CompressedNormal());
        m_has_rgb = true;
        invalidateCache();
    }

    void Cloud::addPoints(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z)
    {
        size_t old_size = m_xyz->size();
        size_t add_count = x.size();
        size_t new_size = old_size + add_count;

        m_xyz->resize(new_size);

        #pragma omp parallel for if (add_count > 10000)
        for (int i = 0; i < add_count; ++i) {
            m_xyz->points[old_size + i] = PointXYZ(x[i], y[i], z[i]);
        }

        if (m_colors) {
            m_colors->resize(new_size, Color::White);
        }
        if (m_normals) {
            m_normals->resize(new_size);
        }

        invalidateCache();
    }

    void Cloud::push_back(const PointXYZRGBN& pt)
    {
        m_xyz->push_back(PointXYZ(pt.x, pt.y, pt.z));
        if (m_colors) {
            m_colors->push_back(RGB(pt.r, pt.g, pt.b));
            m_has_rgb = true;
        }
        if (m_normals) {
            CompressedNormal cn;
            cn.set(Eigen::Vector3f(pt.normal_x, pt.normal_y, pt.normal_z));
            m_normals->push_back(cn);
            m_has_normals = true;
        }
        invalidateCache();
    }

    // ===== 属性管理 =====
    bool Cloud::hasColors() const
    {
        return m_colors != nullptr && !m_colors->empty();
    }

    bool Cloud::hasNormals() const
    {
        return m_normals != nullptr && !m_normals->empty();
    }

    void Cloud::enableColors()
    {
        if (!m_colors) {
            m_colors = std::make_unique<std::vector<RGB>>();
            m_colors->resize(m_xyz->size(), Color::White);
        }
    }

    void Cloud::enableNormals()
    {
        if (!m_normals) {
            m_normals = std::make_unique<std::vector<CompressedNormal>>();
            m_normals->resize(m_xyz->size());
        }
    }

    void Cloud::disableColors()
    {
        m_colors.reset();
        m_has_rgb = false;
        invalidateCache();
    }

    void Cloud::disableNormals()
    {
        m_normals.reset();
        m_has_normals = false;
        invalidateCache();
    }

    // ===== PCL 兼容层（带缓存）=====
    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::toPCL_XYZRGB() const
    {
        if (m_cache_valid && m_cached_xyzrgb) {
            return m_cached_xyzrgb;
        }

        // 限制转换大小，避免内存不足（1.6GB = 1亿点 × 16字节）
        constexpr size_t MAX_CONVERT_POINTS = 50000000;  // 5000万点
        if (m_xyz->size() > MAX_CONVERT_POINTS) {
            // 对超大点云，返回空指针或抛出异常
            // 这里返回一个小的空点云，避免崩溃
            m_cached_xyzrgb = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
            m_cache_valid = true;
            return m_cached_xyzrgb;
        }

        m_cached_xyzrgb = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
        size_t n = m_xyz->size();
        m_cached_xyzrgb->resize(n);

        #pragma omp parallel for if (n > 100000)
        for (int i = 0; i < n; ++i) {
            const auto& src = m_xyz->points[i];
            auto& dst = m_cached_xyzrgb->points[i];

            dst.x = src.x; dst.y = src.y; dst.z = src.z;

            if (m_colors && i < m_colors->size()) {
                dst.r = (*m_colors)[i].r;
                dst.g = (*m_colors)[i].g;
                dst.b = (*m_colors)[i].b;
            } else {
                dst.r = dst.g = dst.b = 255;
            }
        }

        m_cache_valid = true;
        return m_cached_xyzrgb;
    }

    pcl::PointCloud<PointXYZRGBN>::Ptr Cloud::toPCL_XYZRGBN() const
    {
        if (m_cache_valid && m_cached_xyzrgbn) {
            return m_cached_xyzrgbn;
        }

        // 限制转换大小，避免内存不足（2GB = 1亿点 × 20字节）
        constexpr size_t MAX_CONVERT_POINTS = 50000000;  // 5000万点
        if (m_xyz->size() > MAX_CONVERT_POINTS) {
            m_cached_xyzrgbn = std::make_shared<pcl::PointCloud<PointXYZRGBN>>();
            m_cache_valid = true;
            return m_cached_xyzrgbn;
        }

        m_cached_xyzrgbn = std::make_shared<pcl::PointCloud<PointXYZRGBN>>();
        size_t n = m_xyz->size();
        m_cached_xyzrgbn->resize(n);

        #pragma omp parallel for if (n > 100000)
        for (int i = 0; i < n; ++i) {
            const auto& src = m_xyz->points[i];
            auto& dst = m_cached_xyzrgbn->points[i];

            dst.x = src.x; dst.y = src.y; dst.z = src.z;

            if (m_colors && i < m_colors->size()) {
                dst.r = (*m_colors)[i].r;
                dst.g = (*m_colors)[i].g;
                dst.b = (*m_colors)[i].b;
            } else {
                dst.r = dst.g = dst.b = 255;
            }

            if (m_normals && i < m_normals->size()) {
                Eigen::Vector3f n = (*m_normals)[i].get();
                dst.normal_x = n.x();
                dst.normal_y = n.y();
                dst.normal_z = n.z();
            } else {
                dst.normal_x = dst.normal_y = dst.normal_z = 0;
            }
        }

        m_cache_valid = true;
        return m_cached_xyzrgbn;
    }

    void Cloud::clearConversionCache()
    {
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_cache_valid = false;
    }

    // ===== 渲染接口 =====
    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::getRenderCloud() const
    {
        if (m_render_cache_valid && m_render_cloud) {
            return m_render_cloud;
        }

        m_render_cloud = toPCL_XYZRGB();
        m_render_cache_valid = true;
        return m_render_cloud;
    }

    void Cloud::invalidateRenderCache()
    {
        m_render_cloud.reset();
        m_render_cache_valid = false;
    }

    // ===== 预览点云 =====
    void Cloud::generatePreview(int target_points)
    {
        if (size() == 0) {
            m_preview = nullptr;
            return;
        }

        // 如果点云较小，不需要预览（直接使用完整点云）
        // 但避免进行完整的内存复制，只在真正需要渲染时才转换
        if (size() <= target_points) {
            m_preview = nullptr;  // 标记为不使用预览
            return;
        }

        m_preview = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
        m_preview->reserve(target_points);

        int step = size() / target_points;
        if (step < 1) step = 1;

        for (size_t i = 0; i < size(); i += step) {
            PointXYZRGB p;
            p.x = m_xyz->points[i].x;
            p.y = m_xyz->points[i].y;
            p.z = m_xyz->points[i].z;

            if (m_colors && i < m_colors->size()) {
                p.r = (*m_colors)[i].r;
                p.g = (*m_colors)[i].g;
                p.b = (*m_colors)[i].b;
            } else {
                p.r = p.g = p.b = 255;
            }
            m_preview->push_back(p);
        }
    }

    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::getPreviewCloud() const
    {
        if (!m_preview) return toPCL_XYZRGB();
        return m_preview;
    }

    // ===== 颜色设置 =====
    void Cloud::setCloudColor(const RGB& rgb)
    {
        if (!m_has_backup) backupColors();

        if (!m_colors) enableColors();
        if (m_colors->size() != size()) m_colors->resize(size());

        #pragma omp parallel for
        for (int i = 0; i < size(); ++i) {
            (*m_colors)[i] = rgb;
        }

        m_has_rgb = true;
        m_color_modified = true;
        invalidateCache();

        if (m_preview) generatePreview(static_cast<int>(m_preview->size()));
    }

    void Cloud::setCloudColor(const QString& axis)
    {
        if (empty()) return;
        if (!m_has_backup) backupColors();

        if (!m_colors) enableColors();
        if (m_colors->size() != size()) m_colors->resize(size());

        float min_v = 0.0f, max_v = 0.0f;

        if (axis == "x") { min_v = m_min.x; max_v = m_max.x; }
        else if (axis == "y") { min_v = m_min.y; max_v = m_max.y; }
        else if (axis == "z") { min_v = m_min.z; max_v = m_max.z; }
        else return;

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;

        auto hsv2rgb = [](float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
            float c = v * s;
            float x = c * (1 - std::abs(std::fmod(h * 6, 2.0f) - 1));
            float m = v - c;
            float r_ = 0, g_ = 0, b_ = 0;

            if (h < 1.0/6.0) { r_=c; g_=x; b_=0; }
            else if (h < 2.0/6.0) { r_=x; g_=c; b_=0; }
            else if (h < 3.0/6.0) { r_=0; g_=c; b_=x; }
            else if (h < 4.0/6.0) { r_=0; g_=x; b_=c; }
            else if (h < 5.0/6.0) { r_=x; g_=0; b_=c; }
            else { r_=c; g_=0; b_=x; }

            r = static_cast<uint8_t>((r_ + m) * 255);
            g = static_cast<uint8_t>((g_ + m) * 255);
            b = static_cast<uint8_t>((b_ + m) * 255);
        };

        #pragma omp parallel for
        for (int i = 0; i < size(); ++i) {
            float val = 0.0f;
            if (axis == "x") val = m_xyz->points[i].x;
            else if (axis == "y") val = m_xyz->points[i].y;
            else val = m_xyz->points[i].z;

            float norm = (val - min_v) / range;
            float hue = (1.0f - norm) * 0.66f;

            hsv2rgb(hue, 1.0f, 1.0f, (*m_colors)[i].r, (*m_colors)[i].g, (*m_colors)[i].b);
        }

        m_has_rgb = true;
        m_color_modified = true;
        invalidateCache();

        if (m_preview) generatePreview(static_cast<int>(m_preview->size()));
    }

    // ===== 标量场 =====
    void Cloud::addScalarField(const QString& name, std::vector<float>&& data)
    {
        if (data.size() != size()) return;
        m_scalar_fields[name] = std::move(data);
    }

    void Cloud::addScalarField(const QString& name, const std::vector<float>& data)
    {
        if (data.size() != size()) return;
        m_scalar_fields[name] = data;
    }

    bool Cloud::removeScalarField(const QString& name)
    {
        return m_scalar_fields.remove(name) > 0;
    }

    void Cloud::clearScalarFields()
    {
        m_scalar_fields.clear();
    }

    bool Cloud::hasScalarField(const QString& name) const
    {
        return m_scalar_fields.contains(name);
    }

    QStringList Cloud::getScalarFieldNames() const
    {
        return m_scalar_fields.keys();
    }

    const std::vector<float>* Cloud::getScalarField(const QString& name) const
    {
        auto it = m_scalar_fields.find(name);
        return (it != m_scalar_fields.end()) ? &it.value() : nullptr;
    }

    std::vector<float>* Cloud::getScalarField(const QString& name)
    {
        auto it = m_scalar_fields.find(name);
        return (it != m_scalar_fields.end()) ? &it.value() : nullptr;
    }

    void Cloud::updateColorByField(const QString& field_name)
    {
        if (!hasScalarField(field_name)) return;

        if (s_jet_lut.empty()) initColorTable();
        if (!m_has_backup) backupColors();

        const std::vector<float>& data = m_scalar_fields[field_name];

        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;

        #pragma omp parallel
        {
            float local_min = FLT_MAX;
            float local_max = -FLT_MAX;
        #pragma omp for
            for (int i = 0; i < data.size(); ++i) {
                float v = data[i];
                if (v < local_min) local_min = v;
                if (v > local_max) local_max = v;
            }
        #pragma omp critical
            {
                if (local_min < min_v) min_v = local_min;
                if (local_max > max_v) max_v = local_max;
            }
        }

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;

        if (!m_colors) enableColors();
        if (m_colors->size() != size()) m_colors->resize(size());

        const float* pData = data.data();
        const float* pLut = s_jet_lut.data();
        int lut_size = 1024;

        #pragma omp parallel for
        for (int i = 0; i < size(); ++i) {
            float norm = (pData[i] - min_v) / range;
            if (norm < 0.0f) norm = 0.0f;
            if (norm > 1.0f) norm = 1.0f;

            int idx = static_cast<int>(norm * (lut_size - 1));

            float lutVal = pLut[idx];
            uint32_t packed = *reinterpret_cast<const uint32_t*>(&lutVal);

            (*m_colors)[i].r = (packed >> 16) & 0xFF;
            (*m_colors)[i].g = (packed >> 8) & 0xFF;
            (*m_colors)[i].b = packed & 0xFF;
        }

        m_has_rgb = true;
        m_color_modified = true;
        invalidateCache();

        if (m_preview) generatePreview(static_cast<int>(m_preview->size()));
    }

    void Cloud::initColorTable()
    {
        if (!s_jet_lut.empty()) return;
        s_jet_lut.resize(1024);

        for (int i = 0; i < 1024; ++i) {
            float v = (float)i / 1023.0f;
            uint8_t r = 0, g = 0, b = 0;

            if (v < 0.25f) { r=0; g=(uint8_t)(255*4*v); b=255; }
            else if (v < 0.5f) { r=0; g=255; b=(uint8_t)(255*(1-4*(v-0.25))); }
            else if (v < 0.75f) { r=(uint8_t)(255*4*(v-0.5)); g=255; b=0; }
            else { r=255; g=(uint8_t)(255*(1-4*(v-0.75))); b=0; }

            uint32_t packed = (r << 16) | (g << 8) | b;
            s_jet_lut[i] = *reinterpret_cast<float*>(&packed);
        }
    }

    // ===== 拷贝/克隆 =====
    Cloud::Ptr Cloud::clone() const
    {
        Cloud::Ptr result(new Cloud);
        result->copyFrom(*this);
        return result;
    }

    void Cloud::copyFrom(const Cloud& other)
    {
        *this->m_xyz = *other.m_xyz;

        if (other.m_colors) {
            m_colors = std::make_unique<std::vector<RGB>>(*other.m_colors);
        } else {
            m_colors.reset();
        }

        if (other.m_normals) {
            m_normals = std::make_unique<std::vector<CompressedNormal>>(*other.m_normals);
        } else {
            m_normals.reset();
        }

        m_scalar_fields = other.m_scalar_fields;

        m_id = other.m_id;
        m_info = other.m_info;
        m_box = other.m_box;
        m_box_rgb = other.m_box_rgb;
        m_normals_rgb = other.m_normals_rgb;
        m_point_size = other.m_point_size;
        m_opacity = other.m_opacity;
        m_resolution = other.m_resolution;
        m_min = other.m_min;
        m_max = other.m_max;
        m_global_shift = other.m_global_shift;
        m_has_rgb = other.m_has_rgb;
        m_has_normals = other.m_has_normals;
        m_type = other.m_type;

        if (other.m_original_colors) {
            m_original_colors = std::make_unique<std::vector<RGB>>(*other.m_original_colors);
        } else {
            m_original_colors.reset();
        }
        m_has_backup = other.m_has_backup;
        m_color_modified = other.m_color_modified;

        m_preview.reset();
        invalidateCache();
    }

    // ===== 合并点云 =====
    void Cloud::append(const Cloud& other)
    {
        if (other.empty()) return;

        size_t original_size = size();
        size_t other_size = other.size();
        size_t final_size = original_size + other_size;

        *m_xyz += *other.m_xyz;

        bool thisRGB = m_has_rgb && m_colors && !m_colors->empty();
        bool otherRGB = other.m_has_rgb && other.m_colors && !other.m_colors->empty();

        if (thisRGB || otherRGB) {
            if (!thisRGB) {
                enableColors();
                m_colors->resize(original_size, Color::White);
            }

            if (otherRGB) {
                m_colors->insert(m_colors->end(), other.m_colors->begin(), other.m_colors->end());
            } else {
                m_colors->resize(final_size, Color::White);
            }
            m_has_rgb = true;
        }

        bool thisNorm = m_has_normals && m_normals && !m_normals->empty();
        bool otherNorm = other.m_has_normals && other.m_normals && !other.m_normals->empty();

        if (thisNorm || otherNorm) {
            if (!thisNorm) {
                enableNormals();
                m_normals->resize(original_size);
            }

            if (otherNorm) {
                m_normals->insert(m_normals->end(), other.m_normals->begin(), other.m_normals->end());
            } else {
                m_normals->resize(final_size);
            }
            m_has_normals = true;
        }

        QStringList myFields = m_scalar_fields.keys();
        QStringList otherFields = other.m_scalar_fields.keys();

        QSet<QString> allFields;
        for(const auto& f : myFields) allFields.insert(f);
        for(const auto& f : otherFields) allFields.insert(f);

        for (const QString& field : allFields) {
            std::vector<float>& targetVec = m_scalar_fields[field];
            bool iHave = myFields.contains(field);
            bool otherHas = otherFields.contains(field);

            targetVec.reserve(final_size);

            if (!iHave) {
                targetVec.insert(targetVec.begin(), original_size, std::numeric_limits<float>::quiet_NaN());
            }

            if (otherHas) {
                const std::vector<float>& src = other.m_scalar_fields[field];
                targetVec.insert(targetVec.end(), src.begin(), src.end());
            } else {
                targetVec.resize(final_size, std::numeric_limits<float>::quiet_NaN());
            }
        }

        m_preview.reset();
        invalidateCache();
        update();
    }

    Cloud& Cloud::operator+=(const Cloud& rhs)
    {
        append(rhs);
        return *this;
    }

    // ===== 更新 =====
    void Cloud::update()
    {
        if (empty()) {
            m_min.x = m_min.y = m_min.z = 0;
            m_max.x = m_max.y = m_max.z = 0;
            m_type = CLOUD_TYPE_XYZ;
            return;
        }

        pcl::getMinMax3D(*m_xyz, m_min, m_max);

        Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
        Eigen::Vector3f whd = m_max.getVector3fMap() - m_min.getVector3fMap();

        m_box.width = whd.x();
        m_box.height = whd.y();
        m_box.depth = whd.z();
        m_box.pose = Eigen::Affine3f::Identity();
        m_box.pose.translation() = center;
        m_box.translation = center;
        m_box.rotation = Eigen::Quaternionf::Identity();

        if (m_has_normals && m_has_rgb) m_type = CLOUD_TYPE_XYZRGBN;
        else if (m_has_normals) m_type = CLOUD_TYPE_XYZN;
        else if (m_has_rgb) m_type = CLOUD_TYPE_XYZRGB;
        else m_type = CLOUD_TYPE_XYZ;

        // 对于大点云，跳过分辨率计算以避免内存问题
        if (size() > 0 && size() < 10000000) {  // 限制在1000万点以内
            int num_samples = (size() < 1000) ? static_cast<int>(size()) : 1000;
            m_resolution = 0.0f;
            int valid_points = 0;

            pcl::search::KdTree<PointXYZ> tree;
            tree.setInputCloud(m_xyz);

            std::vector<int> indices(2);
            std::vector<float> sqr_dists(2);

            for(int i=0; i<num_samples; ++i) {
                int idx = rand() % size();
                if (!std::isfinite(m_xyz->points[idx].x)) continue;

                if (tree.nearestKSearch(idx, 2, indices, sqr_dists) == 2) {
                    m_resolution += sqrt(sqr_dists[1]);
                    valid_points++;
                }
            }
            if (valid_points > 0) m_resolution /= valid_points;
        } else {
            m_resolution = 0.0f;  // 大点云默认分辨率
        }
    }

    // ===== 颜色备份/恢复 =====
    void Cloud::backupColors()
    {
        if (m_has_backup || !m_colors) return;

        m_original_colors = std::make_unique<std::vector<RGB>>(*m_colors);
        m_has_backup = true;
    }

    void Cloud::restoreColors()
    {
        if (!m_has_backup || !m_original_colors) return;

        if (!m_colors) enableColors();
        *m_colors = *m_original_colors;

        m_color_modified = false;
        m_has_rgb = !m_colors->empty();
        invalidateCache();

        if (m_preview) {
            if (m_preview->size() != m_xyz->size()) {
                generatePreview(static_cast<int>(m_preview->size()));
            } else {
                m_preview = toPCL_XYZRGB();
            }
        }
    }

    // ===== 其他 =====
    Eigen::Vector3d Cloud::getGlobalPoint(size_t index) const
    {
        if (index >= size()) return Eigen::Vector3d::Zero();
        const auto& p = m_xyz->points[index];
        return Eigen::Vector3d(p.x, p.y, p.z) + m_global_shift;
    }

    PointXYZRGBN Cloud::getPoint(size_t index) const
    {
        PointXYZRGBN pt;
        if (index >= size()) return pt;

        const auto& xyz_pt = m_xyz->points[index];
        pt.x = xyz_pt.x;
        pt.y = xyz_pt.y;
        pt.z = xyz_pt.z;

        if (m_colors && index < m_colors->size()) {
            pt.r = (*m_colors)[index].r;
            pt.g = (*m_colors)[index].g;
            pt.b = (*m_colors)[index].b;
        } else {
            pt.r = pt.g = pt.b = 255;
        }

        if (m_normals && index < m_normals->size()) {
            Eigen::Vector3f n = (*m_normals)[index].get();
            pt.normal_x = n.x();
            pt.normal_y = n.y();
            pt.normal_z = n.z();
        } else {
            pt.normal_x = pt.normal_y = pt.normal_z = 0;
        }

        return pt;
    }

    std::uint32_t Cloud::getPointColorForSave(size_t index) const
    {
        if (!m_has_rgb || !m_colors || index >= m_colors->size()) {
            return (255<<16) | (255<<8) | 255;
        }

        const RGB& c = (*m_colors)[index];
        return (c.r << 16) | (c.g << 8) | c.b;
    }

    void Cloud::removeInvalidPoints()
    {
        if (m_xyz->is_dense) return;

        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*m_xyz, *m_xyz, indices);

        size_t new_size = m_xyz->size();

        if (m_has_rgb && m_colors && m_colors->size() != new_size) {
            std::vector<RGB> new_colors;
            new_colors.reserve(new_size);
            for (int idx : indices) new_colors.push_back((*m_colors)[idx]);
            *m_colors = std::move(new_colors);
        }

        if (m_has_normals && m_normals && m_normals->size() != new_size) {
            std::vector<CompressedNormal> new_normals;
            new_normals.reserve(new_size);
            for (int idx : indices) new_normals.push_back((*m_normals)[idx]);
            *m_normals = std::move(new_normals);
        }

        for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
            std::vector<float>& data = it.value();
            if (data.size() != new_size) {
                std::vector<float> new_data;
                new_data.reserve(new_size);
                for (int idx : indices) new_data.push_back(data[idx]);
                it.value() = std::move(new_data);
            }
        }

        invalidateCache();
        update();
    }

    // ===== PCL 兼容层 - 从 PCL 点云构造 =====
    Cloud::Ptr Cloud::fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud)
    {
        Cloud::Ptr result(new Cloud);
        size_t n = pcl_cloud.size();

        result->m_xyz->resize(n);
        result->enableColors();
        result->enableNormals();

        #pragma omp parallel for if (n > 100000)
        for (int i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            result->m_xyz->points[i].x = src.x;
            result->m_xyz->points[i].y = src.y;
            result->m_xyz->points[i].z = src.z;

            (*result->m_colors)[i] = RGB(src.r, src.g, src.b);

            CompressedNormal cn;
            cn.set(Eigen::Vector3f(src.normal_x, src.normal_y, src.normal_z));
            (*result->m_normals)[i] = cn;
        }

        result->m_has_rgb = true;
        result->m_has_normals = true;

        return result;
    }

    Cloud::Ptr Cloud::fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB>& pcl_cloud)
    {
        Cloud::Ptr result(new Cloud);
        size_t n = pcl_cloud.size();

        result->m_xyz->resize(n);
        result->enableColors();

        #pragma omp parallel for if (n > 100000)
        for (int i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            result->m_xyz->points[i].x = src.x;
            result->m_xyz->points[i].y = src.y;
            result->m_xyz->points[i].z = src.z;

            (*result->m_colors)[i] = RGB(src.r, src.g, src.b);
        }

        result->m_has_rgb = true;

        return result;
    }

    // ===== 辅助方法 =====
    void Cloud::invalidateCache()
    {
        m_cache_valid = false;
        m_render_cache_valid = false;
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_render_cloud.reset();
    }

    void Cloud::ensureSizeConsistency()
    {
        size_t n = m_xyz->size();

        if (m_colors && m_colors->size() != n) {
            m_colors->resize(n, Color::White);
        }

        if (m_normals && m_normals->size() != n) {
            m_normals->resize(n);
        }
    }
}
