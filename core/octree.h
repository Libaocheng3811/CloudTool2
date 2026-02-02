//
// Created by LBC on 2026/1/26.
//

#ifndef CLOUDTOOL2_OCTREE_H
#define CLOUDTOOL2_OCTREE_H

#include "cloudtype.h"
#include <vector>
#include <memory>
#include <array>
#include <QMap>

namespace ct{
    /**
     * @brief 数据块 (Data Block) - 八叉树叶子负载
     */
    class CloudBlock
    {
    public:
        using Ptr = std::shared_ptr<CloudBlock>;

        CloudBlock() {
            m_box.width = m_box.height = m_box.depth = 0;
        }

        // --- 核心数据存储 ---
        std::vector<pcl::PointXYZ> m_points;
        std::unique_ptr<std::vector<ct::RGB>> m_colors;
        std::unique_ptr<std::vector<ct::RGB>> m_backup_colors;
        std::unique_ptr<std::vector<ct::CompressedNormal>> m_normals;
        QMap<QString, std::vector<float>> m_scalar_fields;

        // --- 空间属性 ---
        Box m_box;

        // --- 渲染状态 ---
        /**
        * @brief VTK 数据缓存
        * @details 实际上存储的是 vtkSmartPointer<vtkPolyData>。
        * 使用 void* 或 shared_ptr<void> 避免在此处引入 vtk 头文件，降低编译依赖。
        * View 层负责 cast 和管理。
        */
        std::shared_ptr<void> m_vtk_polydata;
        bool m_is_visible = true;
        bool m_is_dirty = true;
        size_t m_gpu_handle = 0;

        // --- 辅助方法 ---
        size_t size() const { return m_points.size(); }
        bool empty() const { return m_points.empty(); }

        /**
         * @brief 添加一个点，并保持所有属性向量的大小同步
         * @param pt 点坐标
         */
        void addPoint(const pcl::PointXYZ& pt) {
            m_points.push_back(pt);
            if (m_colors) m_colors->push_back(ct::Color::White);

            if (m_normals) m_normals->push_back(ct::CompressedNormal());

            if (!m_scalar_fields.isEmpty()) {
                for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
                    it.value().push_back(0.0f);
                }
            }
            m_is_dirty = true;
        }

        /**
         * @brief 添加一个带完整属性的点 (重载版本，方便流式加载时直接赋值)
         */
        void addPoint(const pcl::PointXYZ& pt,
                      const ct::RGB* color,
                      const ct::CompressedNormal* normal,
                      const QMap<QString, float>* scalars = nullptr)
        {
            m_points.push_back(pt);

            if (color && !m_colors) {
                m_colors = std::make_unique<std::vector<RGB>>();
                if (!m_points.empty()) {
                    m_colors->resize(m_points.size() - 1, ct::Color::White);
                }
            }
            if (m_colors){
                m_colors->push_back(color ? *color : ct::Color::White);
            }

            if (normal && !m_normals) {
                m_normals = std::make_unique<std::vector<CompressedNormal>>();
                if (!m_points.empty()) {
                    m_normals->resize(m_points.size() - 1);
                }
            }
            if (m_normals) {
                m_normals->push_back(normal ? *normal : ct::CompressedNormal());
            }

            // 同步标量场
            if (!m_scalar_fields.isEmpty()) {
                for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
                    float val = 0.0f;
                    // 如果传入了标量值且包含当前字段，则使用传入值
                    if (scalars && scalars->contains(it.key())) {
                        val = scalars->value(it.key());
                    }
                    it.value().push_back(val);
                }
            }
            m_is_dirty = true;
        }

        /**
         * @brief 清空数据
         */
        void clear() {
            m_points.clear();

            if (m_colors) m_colors->clear();
            if (m_normals) m_normals->clear();
            if (m_backup_colors) m_backup_colors->clear();
            for (auto it = m_scalar_fields.begin(); it != m_scalar_fields.end(); ++it) {
                it.value().clear();
            }

            m_vtk_polydata.reset();
            m_is_dirty = true;
        }

        /**
         * @brief 注册一个新的标量场 (如果不存在)
         * 当在读取文件头发现新字段时调用此函数
         */
        void registerScalarField(const QString& name) {
            if (!m_scalar_fields.contains(name)) {
                // 创建新向量，并预分配内存
                std::vector<float>& vec = m_scalar_fields[name];

                // 如果当前 Block 已经有点了，需要补齐 0，保持对齐
                if (!m_points.empty()) {
                    vec.resize(m_points.size(), 0.0f);
                }
            }
        }

        void markDirty() { m_is_dirty = true; }
    };

    /**
     * @brief 八叉树节点 (Octree Node)
     */
    class OctreeNode
    {
    public:
        using Ptr = std::shared_ptr<OctreeNode>;

        OctreeNode(const Box& box, int depth, OctreeNode* parent = nullptr)
                : m_box(box), m_depth(depth), m_parent(parent)
        {
            for(int i=0; i<8; ++i) m_children[i] = nullptr;
        }

        ~OctreeNode() {
            for(int i=0; i<8; ++i) {
                if(m_children[i]) delete m_children[i];
            }
        }

        // --- 结构关系 ---
        OctreeNode* m_parent = nullptr;
        OctreeNode* m_children[8];

        // --- 空间属性 ---
        Box m_box;
        int m_depth;
        size_t m_total_points_in_node = 0;

        // --- 数据负载 (叶子节点持有) ---
        CloudBlock::Ptr m_block = nullptr;

        // --- LOD 数据 (内部节点持有) ---
        std::vector<pcl::PointXYZRGB> m_lod_points;

        std::shared_ptr<void> m_vtk_lod_polydata;
        bool m_lod_dirty = true;

        void clearLOD() {
            m_lod_points.clear();
            m_lod_points.shrink_to_fit();
            m_vtk_lod_polydata.reset();
            m_lod_dirty = true;
        }

        // --- 方法 ---
        bool isLeaf() const { return m_block != nullptr; }

        bool hasChildren() const {
            for(int i=0; i<8; ++i) if(m_children[i]) return true;
            return false;
        }

        // 计算点所属子象限索引
        int getChildIndex(const pcl::PointXYZ& pt) const {
            float cx = m_box.translation.x();
            float cy = m_box.translation.y();
            float cz = m_box.translation.z();
            int index = 0;
            if (pt.x >= cx) index |= 1;
            if (pt.y >= cy) index |= 2;
            if (pt.z >= cz) index |= 4;
            return index;
        }
    };
} // namespace ct

#endif //CLOUDTOOL2_OCTREE_H
