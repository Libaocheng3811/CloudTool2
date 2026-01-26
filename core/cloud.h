#ifndef CLOUDTOOL2_CLOUD_H
#define CLOUDTOOL2_CLOUD_H

#include "exports.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <QFileInfo>
#include <QString>
#include <QMap>

#include <QMetaType>
#include <memory>
#include <vector>

#define CLOUD_TYPE_XYZ       "xyz"
#define CLOUD_TYPE_XYZRGB    "XYZRGB"
#define CLOUD_TYPE_XYZN      "XYZNormal"
#define CLOUD_TYPE_XYZRGBN   "XYZRGBNormal"

#define BOX_PRE_FLAG         "-box"
#define NORMALS_PRE_FLAG     "-normals"

#define PREVIEW_LIMIT       8000000

namespace ct
{
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

    class CT_EXPORT Cloud :public std::enable_shared_from_this<Cloud>
    {
    public:
        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        // ===== 点引用结构（用于安全访问单个点）=====
        struct PointRef
        {
            float& x;
            float& y;
            float& z;
            bool has_rgb;
            bool has_normal;
            RGB* rgb;
            CompressedNormal* normal;

            PointRef(float& x_, float& y_, float& z_, bool has_rgb_, bool has_normal_,
                    RGB* rgb_, CompressedNormal* normal_)
                : x(x_), y(y_), z(z_), has_rgb(has_rgb_), has_normal(has_normal_),
                  rgb(rgb_), normal(normal_) {}

            void setColor(uint8_t r, uint8_t g, uint8_t b) {
                if (rgb) { rgb->r = r; rgb->g = g; rgb->b = b; }
            }
            void setNormal(const Eigen::Vector3f& n) {
                if (normal) normal->set(n);
            }
            void setNormal(float nx, float ny, float nz) {
                if (normal) normal->set(Eigen::Vector3f(nx, ny, nz));
            }
        };

        struct ConstPointRef
        {
            const float& x;
            const float& y;
            const float& z;
            bool has_rgb;
            bool has_normal;
            const RGB* rgb;
            const CompressedNormal* normal;

            ConstPointRef(const float& x_, const float& y_, const float& z_,
                         bool has_rgb_, bool has_normal_,
                         const RGB* rgb_, const CompressedNormal* normal_)
                : x(x_), y(y_), z(z_), has_rgb(has_rgb_), has_normal(has_normal_),
                  rgb(rgb_), normal(normal_) {}

            Eigen::Vector3f getNormal() const {
                return normal ? normal->get() : Eigen::Vector3f::Zero();
            }
        };

        // ===== 构造/析构 =====
        Cloud();
        ~Cloud();

        // 禁用拷贝，启用移动
        Cloud(const Cloud& other) = delete;
        Cloud& operator=(const Cloud& other) = delete;
        Cloud(Cloud&& other) noexcept;
        Cloud& operator=(Cloud&& other) noexcept;

        // ===== 容量接口 =====
        size_t size() const;
        bool empty() const;
        void reserve(size_t n);
        void resize(size_t n);
        void clear();

        // ===== 点访问 =====
        PointRef operator[](size_t index);
        ConstPointRef operator[](size_t index) const;
        PointRef at(size_t index);
        ConstPointRef at(size_t index) const;

        // ===== 批量添加 =====
        void addPoint(float x, float y, float z);
        void addPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
        void addPoints(const std::vector<float>& x, const std::vector<float>& y, const std::vector<float>& z);

        // 旧接口，兼容性保留
        void push_back(const PointXYZRGBN& pt);

        // ===== 属性管理 =====
        bool hasColors() const;
        bool hasNormals() const;

        /**
         * @brief Get compressed normal data (for rendering/viewing only)
         * @return Pointer to compressed normals vector, or nullptr if not available
         */
        const std::vector<CompressedNormal>* getNormalsData() const { return m_normals.get(); }

        void enableColors();
        void enableNormals();
        void disableColors();    // 释放颜色内存
        void disableNormals();   // 释放法线内存

        // ===== PCL 兼容层 =====
        // 零拷贝：返回内部 PCL 点云（只读）
        pcl::PointCloud<PointXYZ>::Ptr toPCL_XYZ() const { return m_xyz; }

        // 直接访问点云数据（用于遍历点）
        const pcl::PointCloud<PointXYZ>* getXYZCloud() const { return m_xyz.get(); }

        // 按需转换：带缓存
        pcl::PointCloud<PointXYZRGB>::Ptr toPCL_XYZRGB() const;
        pcl::PointCloud<PointXYZRGBN>::Ptr toPCL_XYZRGBN() const;

        // 清除转换缓存
        void clearConversionCache();

        // ===== 渲染接口 =====
        // 直接返回渲染用点云（避免重复转换）
        pcl::PointCloud<PointXYZRGB>::Ptr getRenderCloud() const;
        void invalidateRenderCache();

        /**
         * @brief 生成预览点云,800万点云
         */
        void generatePreview(int target_points = PREVIEW_LIMIT);

        /**
         * @brief 获取预览点云
         * @return
         */
        pcl::PointCloud<PointXYZRGB>::Ptr getPreviewCloud() const;

        ////////////////////////////////////////////////////////////////
        //属性访问
        /**
         * @brief 获取点云ID，设置点云id
         */
        QString id() const {return m_id;}
        void setId(const QString& id) {m_id = id;}

        /**
         * @brief 获取点云包围盒ID，设置点云包围盒ID
         */
        QString boxId() const {return m_id + BOX_PRE_FLAG;}
        QString normalId() const {return m_id + NORMALS_PRE_FLAG;}

        /**
         * @brief 获取包围盒, 设置包围盒
         */
        Box box() const {return m_box;}
        void setBox(const Box& box) {m_box = box;}

        /**
         * @brief 获取包围盒颜色，设置包围盒颜色
         */
        RGB boxColor() const {return m_box_rgb;}
        void setBoxColor(const RGB& rgb) {m_box_rgb = rgb;}

        /**
         * @brief 获取法线颜色，设置法线颜色
         */
        RGB normalColor() const {return m_normals_rgb;}
        void setNormalColor(const RGB& rgb) {m_normals_rgb = rgb;}

        /**
         * @brief 获取点云信息, 设置点云信息
         */
        QFileInfo info() const {return m_info;}
        void setInfo(const QFileInfo& info) {m_info = info;}

        /**
         * @brief 获取点云大小, 设置点云大小
         */
        int pointSize() const {return m_point_size;}
        void setPointSize(int size) {m_point_size = size;}

        /**
         * @brief 获取点云透明度, 0-1, 设置点云透明度
         */
        float opacity() const {return m_opacity;}
        void setOpacity(float opacity) {m_opacity = opacity;}

        /**
         * @brief 获取点云分辨率
         */
        float resolution() const {return m_resolution;}

        /**
         * @brief 获取点云是否带颜色，设置点云是否带颜色
         */
        bool hasRGB() const {return m_has_rgb;}
        void setHasRGB(bool has) {m_has_rgb = has;}

        /**
         * @brief 获取点云是否带法线
         */
        void setHasNormals(bool has) {m_has_normals = has;}

        /**
         * @brief 获取点云类型
         */
        QString type() const {return m_type;}

        /**
         * @brief 获取包围盒中心
         */
        Eigen::Vector3f center() const {return m_box.translation;}

        /**
         * @brief 获取最大点，最小点
         */
        PointXYZ min() const {return m_min;}
        PointXYZ max() const {return m_max;}

        /**
         * @brief 设置全局平移, 获取全局平移
         * @param shift
         */
        void setGlobalShift(const Eigen::Vector3d& shift) { m_global_shift = shift; }
        Eigen::Vector3d getGlobalShift() const { return m_global_shift; }

        /**
         * @brief 获取全局点坐标
         */
        Eigen::Vector3d getGlobalPoint(size_t index) const;

        /**
         * @brief 获取指定索引的点（PointXYZRGBN格式）
         * @param index 点的索引
         * @return 点的PointXYZRGBN表示
         */
        PointXYZRGBN getPoint(size_t index) const;

        /**
         * @brief 获取点云颜色
         */
        void setCloudColor(const RGB& rgb);
        void setCloudColor(const QString& axis);

        /**
        * @brief 获取包围盒体积
        */
        double volume() const { return m_box.width * m_box.height * m_box.depth; }

        //////////////////////////////////////////////////////////////////////////////////
        // 标量场相关
        /**
         * @brief 添加标量场（移动语义）
         * @param name 字段名称
         * @param data 字段数据
         */
        void addScalarField(const QString& name, std::vector<float>&& data);
        void addScalarField(const QString& name, const std::vector<float>& data);

        /**
         * @brief 移除标量场
         */
        bool removeScalarField(const QString& name);

        /**
         * @brief 清空所有标量场
         */
        void clearScalarFields();

        /**
         * @brief 是否存在标量场
         */
        bool hasScalarField(const QString& name) const;

        /**
         * @brief 获取标量场名称列表
         */
        QStringList getScalarFieldNames() const;

        /**
         * @brief 获取标量场数据
         * @param name 字段名称
         */
        const std::vector<float>* getScalarField(const QString& name) const;
        std::vector<float>* getScalarField(const QString& name);

        /**
         * @brief 更新颜色,根据标量场更新颜色
         * @param field_name 字段名称
         */
        void updateColorByField(const QString& field_name);

        /**
         * @brief 备份颜色
         */
        void backupColors();

        /**
         * @brief 还原颜色
         */
        void restoreColors();

        ////////////////////////////////////////////////////////////////////
        // 核心函数
        /**
         * @brief 更新包围盒，类型，分辨率等信息
         */
        void update();

        /**
         * @brief 深拷贝
         */
        Ptr clone() const;

        /**
         * @brief 添加点云
         */
        void append(const Cloud& cloud);

        Cloud& operator +=(const Cloud& cloud);

        std::uint32_t getPointColorForSave(size_t index) const;
        void removeInvalidPoints();

        // ===== PCL 兼容层 =====
        /**
         * @brief 从 PCL XYZRGBN 点云构造
         */
        static Ptr fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud);

        /**
         * @brief 从 PCL XYZRGB 点云构造
         */
        static Ptr fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB>& pcl_cloud);

        // ===== 受控访问（仅限友元）=====
        // FileIO 需要直接访问内部结构进行高效加载
        friend class FileIO;

    private:
        void initColorTable();
        void copyFrom(const Cloud& other);
        void invalidateCache();
        void ensureSizeConsistency();

    private:
        // ===== 核心数据（私有）=====
        pcl::PointCloud<PointXYZ>::Ptr m_xyz;

        // 可选属性（使用 unique_ptr 管理内存）
        std::unique_ptr<std::vector<RGB>> m_colors;
        std::unique_ptr<std::vector<CompressedNormal>> m_normals;

        // 标量场
        QMap<QString, std::vector<float>> m_scalar_fields;

        // ===== 转换缓存（可变）=====
        mutable pcl::PointCloud<PointXYZRGB>::Ptr m_cached_xyzrgb;
        mutable pcl::PointCloud<PointXYZRGBN>::Ptr m_cached_xyzrgbn;
        mutable bool m_cache_valid = false;

        // ===== 渲染缓存 =====
        mutable pcl::PointCloud<PointXYZRGB>::Ptr m_render_cloud;
        mutable bool m_render_cache_valid = false;

        // ===== 预览点云 =====
        pcl::PointCloud<PointXYZRGB>::Ptr m_preview;

        // ===== 元数据 =====
        QString m_id;
        QFileInfo m_info;
        Box m_box;

        RGB m_box_rgb;
        RGB m_normals_rgb;

        int m_point_size = 1;
        float m_opacity = 1.0f;
        float m_resolution = 0.0f;

        PointXYZ m_min;
        PointXYZ m_max;

        Eigen::Vector3d m_global_shift = Eigen::Vector3d::Zero();

        bool m_has_rgb = false;
        bool m_has_normals = false;
        QString m_type = CLOUD_TYPE_XYZ;

        // 颜色备份
        std::unique_ptr<std::vector<RGB>> m_original_colors;
        bool m_has_backup = false;
        bool m_color_modified = false;

        static std::vector<float> s_jet_lut; // 颜色映射表

        std::vector<size_t> m_preview_indices;  // 预览点云索引
    };
}

Q_DECLARE_METATYPE(ct::Cloud::Ptr)

#endif //CLOUDTOOL2_CLOUD_H
