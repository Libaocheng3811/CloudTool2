#ifndef CLOUDTOOL2_CLOUD_H
#define CLOUDTOOL2_CLOUD_H

#include "cloudtype.h"
#include "octree.h"
#include "exports.h"

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

namespace ct
{
    class CT_EXPORT Cloud :public std::enable_shared_from_this<Cloud>
    {
    public:
        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        Cloud();
        ~Cloud();

        // 禁用拷贝，启用移动
        Cloud(const Cloud& other) = delete;
        Cloud& operator=(const Cloud& other) = delete;
        Cloud(Cloud&& other) = delete;
        Cloud& operator=(Cloud&& other) = delete;

        void swap(Cloud& other);

        // ===== 配置接口 =====
        /**
         * @brief 根据点数自动计算最佳配置
         * @param totalPoints 点云总点数
         */
        static CloudConfig calculateAdaptiveConfig(size_t totalPoints);

        /**
         * @brief 自适应完成点云构建
         * @details 统一执行：更新统计 -> 智能计算配置 -> 应用配置 -> 生成LOD
         * 无论点云来源如何，填充完数据后调用一次此函数即可。
         */
        void makeAdaptive();

        /**
         * @brief 设置配置 (需要在 initOctree 之前调用)
         */
        void setConfig(const CloudConfig& config) { m_config = config; }
        const CloudConfig& getConfig() const { return m_config; }

        // ===== 八叉树构建与数据加载 =====
        /**
         * @brief 初始化八叉树空间范围
         * @details 在加载数据前必须调用。如果是文件加载，先读 Header 获取 BBox。
         * @param globalBox 全局包围盒
         * @param maxDepth 最大深度 (建议 8-10)
         */
        void initOctree(const Box& globalBox);

        /**
         * @brief 添加一个点 (自动路由到对应 Block)
         * @note 仅用于少量点编辑或交互
         */
        void addPoint(const PointXYZ& pt, const RGB* color = nullptr, const CompressedNormal* normal = nullptr);

        /**
        * @brief 批量添加点（核心入口）
        * @param pts 点坐标
        * @param colors 颜色 (可选)
        * @param normals 法线 (可选)
        * @param scalars 标量场数据 (可选，Key=字段名, Value=对应点的数值向量)
        */
        void addPoints(const std::vector<PointXYZ>& pts,
                       const std::vector<RGB>* colors = nullptr,
                       const std::vector<CompressedNormal>* normals = nullptr,
                       const QMap<QString, std::vector<float>>* scalars = nullptr);

        void addPoint(const PointXYZRGBN& pt);

        /**
        * @brief 生成八叉树的 LOD (Level of Detail) 数据
        * @details 自底向上遍历树，父节点从子节点中随机采样点作为代理。
         * 建议在 addPoints 完成后调用一次。
        */
        void generateLOD();

        // ===== 核心访问接口 =====
        /**
         * @brief 获取所有数据块 (扁平化列表)
         * @return 块列表，用于遍历处理或非八叉树渲染
         */
        const std::vector<CloudBlock::Ptr>& getBlocks() const { return m_all_blocks; }

        /**
         * @brief 获取八叉树根节点 (用于视锥体剔除和 LOD 遍历)
         */
        OctreeNode* getOctreeRoot() const { return m_octree_root.get(); }

        // ===== 容量接口 =====
        size_t size() const;
        bool empty() const;
        void clear();

        // ===== 属性管理 =====
        bool hasColors() const { return m_has_rgb; };
        bool hasNormals() const { return m_has_normals; };
        void setHasColors(bool has_colors) { m_has_rgb = has_colors; };
        void setHasNormals(bool has_normals) { m_has_normals = has_normals; };

        // 启用属性：会对所有现有 Block 分配内存
        void enableColors();
        void enableNormals();
        void disableColors();
        void disableNormals();

        // ===== 标量场管理  =====
        void addScalarField(const QString& name, const std::vector<float>& data); // 自动拆分数据到 Blocks
        bool removeScalarField(const QString& name);
        void clearScalarFields();
        bool hasScalarField(const QString& name) const;
        QStringList getScalarFieldNames() const;
        const std::vector<float>* getScalarField(const QString& name) const;

        // 根据标量场更新颜色 (遍历所有 Block)
        void updateColorByField(const QString& field_name);

        // ===== 颜色操作 =====
        void setCloudColor(const RGB& rgb);           // 设置纯色
        void setCloudColor(const QString& axis);      //按坐标轴着色
        void updateLODColorRecursive(OctreeNode* node, const RGB& rgb);
        QString currentColorMode() const { return m_current_color_mode; }

        void backupColors();  // 所有 Block 备份当前颜色
        void restoreColors(); // 所有 Block 还原颜色
        bool isColorModified() const { return m_color_modified; }

        // ===== PCL 兼容导出 (拼接模式 - 慎用!) =====
        /**
         * @brief 转换为 PCL 点云 (用于保存文件或无法适配的算法)
         * @warning 这会产生巨大的内存开销，大点云请慎用！
         */
        pcl::PointCloud<PointXYZ>::Ptr toPCL_XYZ() const;
        pcl::PointCloud<PointXYZRGB>::Ptr toPCL_XYZRGB() const;
        pcl::PointCloud<PointXYZRGBN>::Ptr toPCL_XYZRGBN() const;

        /**
        * @brief 从 PCL XYZRGBN 点云构造
        */
        static Ptr fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud);
        static Ptr fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB>& pcl_cloud);
        // ===== 几何与元数据 =====
        void update(); // 更新包围盒、统计信息

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
        * @brief 获取包围盒体积
        */
        double volume() const { return m_box.width * m_box.height * m_box.depth; }

        ////////////////////////////////////////////////////////////////////
       // 算法接口
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

        pcl::PointCloud<PointXYZRGB>::Ptr getRenderCloud() const;
        void invalidateRenderCache();

        friend class FileIO;

    private:
        void initColorTable();
        void copyFrom(const Cloud& other);
        void invalidateCache();
        // 递归插入辅助函数
        CloudBlock* insertPointToOctree(OctreeNode* node, const PointXYZ& pt,
                                 const RGB* color, const CompressedNormal* normal);

        // 节点分裂逻辑
        void splitNode(OctreeNode* node);

        // 递归收集统计信息
        void updateRecursive(OctreeNode* node, PointXYZ& min_pt, PointXYZ& max_pt, size_t& count);

        std::unique_ptr<OctreeNode> cloneOctreeRecursive(const OctreeNode* src_node, OctreeNode* parent);

        /**
        * @brief 递归生成 LOD
        * @return 该节点贡献给父节点的采样点列表
        */
        void generateLODRecursive(OctreeNode* node);

        // 内部辅助：检查当前八叉树结构是否已经发生分裂
        bool isStructureSplit() const;
    private:
        // ===== 核心数据（私有）=====
        // 八叉树根节点
        CloudConfig m_config;
        OctreeNode::Ptr m_octree_root;
        std::vector<CloudBlock::Ptr> m_all_blocks;
        size_t m_point_count = 0;
        int m_max_depth = 8;

        mutable QMap<QString, std::vector<float>> m_scalar_cache; // 标量场导出缓存

        mutable pcl::PointCloud<PointXYZ>::Ptr m_cached_xyz;       // XYZ 缓存
        mutable pcl::PointCloud<PointXYZRGB>::Ptr m_cached_xyzrgb; // XYZRGB 缓存
        mutable pcl::PointCloud<PointXYZRGBN>::Ptr m_cached_xyzrgbn; // XYZRGBN 缓存
        mutable bool m_cache_valid = false; // 缓存有效标志

        mutable pcl::PointCloud<PointXYZRGB>::Ptr m_render_cloud; // 渲染缓存
        mutable bool m_render_cache_valid = false;

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

        bool m_color_modified = false;
        QString m_current_color_mode = "RGB (Default)";

        static std::vector<float> s_jet_lut; // 颜色映射表
    };
}

Q_DECLARE_METATYPE(ct::Cloud::Ptr)

#endif //CLOUDTOOL2_CLOUD_H
