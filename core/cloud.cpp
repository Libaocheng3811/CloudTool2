#include "cloud.h"
#include "common.h"

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <random>
#include <numeric>
#include <queue>
#include <omp.h>

namespace ct
{
    std::vector<float> Cloud::s_jet_lut;

    /**
    * @brief 辅助函数：计算 vector<PointXYZ> 的包围盒
    * @note 弥补 PCL 缺少对原生 vector 支持的缺陷
    */
    void getMinMax3D(const std::vector<PointXYZ>& pts, PointXYZ& min_pt, PointXYZ& max_pt)
    {
        if (pts.empty()) {
            min_pt = max_pt = PointXYZ(0, 0, 0);
            return;
        }

        // 初始化为极值
        min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
        max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

        // 简单的线性遍历
        for (const auto& p : pts) {
            if (p.x < min_pt.x) min_pt.x = p.x;
            if (p.x > max_pt.x) max_pt.x = p.x;
            if (p.y < min_pt.y) min_pt.y = p.y;
            if (p.y > max_pt.y) max_pt.y = p.y;
            if (p.z < min_pt.z) min_pt.z = p.z;
            if (p.z > max_pt.z) max_pt.z = p.z;
        }
    }

    // ===== 构造/析构 =====
    Cloud::Cloud()
        : m_point_count(0),
          m_max_depth(8),
          m_id("cloud"),
          m_box_rgb(Color::White),
          m_normals_rgb(Color::White)
    {
    }

    Cloud::~Cloud() = default;

    void Cloud::swap(ct::Cloud &other) {
        // 使用 std::swap 交换所有成员
        std::swap(m_octree_root, other.m_octree_root);
        std::swap(m_all_blocks, other.m_all_blocks);
        std::swap(m_point_count, other.m_point_count);
        std::swap(m_max_depth, other.m_max_depth);
        std::swap(m_scalar_cache, other.m_scalar_cache);

        // 渲染缓存
        std::swap(m_cached_xyz, other.m_cached_xyz);
        std::swap(m_cached_xyzrgb, other.m_cached_xyzrgb);
        std::swap(m_cached_xyzrgbn, other.m_cached_xyzrgbn);
        std::swap(m_cache_valid, other.m_cache_valid);
        std::swap(m_render_cloud, other.m_render_cloud);
        std::swap(m_render_cache_valid, other.m_render_cache_valid);

        // 元数据
        std::swap(m_id, other.m_id);
        std::swap(m_info, other.m_info);
        std::swap(m_box, other.m_box);
        std::swap(m_box_rgb, other.m_box_rgb);
        std::swap(m_normals_rgb, other.m_normals_rgb);
        std::swap(m_point_size, other.m_point_size);
        std::swap(m_opacity, other.m_opacity);
        std::swap(m_resolution, other.m_resolution);
        std::swap(m_min, other.m_min);
        std::swap(m_max, other.m_max);
        std::swap(m_global_shift, other.m_global_shift);
        std::swap(m_has_rgb, other.m_has_rgb);
        std::swap(m_has_normals, other.m_has_normals);
        std::swap(m_type, other.m_type);
        std::swap(m_color_modified, other.m_color_modified);
    }

    void Cloud::initOctree(const ct::Box &globalBox, int maxDepth) {
        // 清空现有数据
        clear();

        m_max_depth = maxDepth;
        m_box = globalBox; // 设置全局包围盒
        m_box.width *= 1.05f;
        m_box.height *= 1.05f;
        m_box.depth *= 1.05f;

        // 创建根节点
        m_octree_root = std::make_shared<OctreeNode>(globalBox, 0, nullptr);

        // 根节点初始时也是叶子节点，拥有一个空的 Block
        m_octree_root->m_block = std::make_shared<CloudBlock>();

        // 记录到扁平化列表中
        m_all_blocks.push_back(m_octree_root->m_block);
    }

    void Cloud::addPoint(const PointXYZ& pt, const RGB* color, const CompressedNormal* normal)
    {
        if (!m_octree_root) {
            Box defaultBox;
            defaultBox.width = 10000.0; defaultBox.height = 10000.0; defaultBox.depth = 10000.0;
            defaultBox.translation = Eigen::Vector3f(0,0,0);
            initOctree(defaultBox);
        }

        // 从根节点开始递归插入
        insertPointToOctree(m_octree_root.get(), pt, color, normal);
        m_point_count++;

        // 标记颜色已被修改 (如果添加了带颜色的点)
        if (color) m_color_modified = true;
    }

    CloudBlock* Cloud::insertPointToOctree(OctreeNode* node, const PointXYZ& pt,
                                    const RGB* color, const CompressedNormal* normal)
    {
        // 如果是叶子节点，直接尝试放入
        if (node->isLeaf()) {
            // 检查容量
            if (node->m_block->size() < MAX_POINTS_PER_BLOCK || node->m_depth >= m_max_depth) {
                // 未满，或者已达到最大深度（无法再分），直接添加
                node->m_block->addPoint(pt, color, normal, nullptr);
                return node->m_block.get();
            } else {
                // 满了且可以分裂 -> 执行分裂
                splitNode(node);
                return insertPointToOctree(node, pt, color, normal);
            }
        }
            // 如果是内部节点，向下路由
        else {
            node->m_total_points_in_node++;
            if (node->m_total_points_in_node % 50 == 0) {
                PointXYZRGB lod_pt;
                lod_pt.x = pt.x; lod_pt.y = pt.y; lod_pt.z = pt.z;
                if (color) { lod_pt.r = color->r; lod_pt.g = color->g; lod_pt.b = color->b; }
                else { lod_pt.r = 255; lod_pt.g = 255; lod_pt.b = 255; }
                node->m_lod_points.push_back(lod_pt);
                node->m_lod_dirty = true; // 标记脏
            }

            int childIdx = node->getChildIndex(pt);

            // 如果对应子节点不存在，按需创建
            if (!node->m_children[childIdx]) {
                // 计算子节点的包围盒
                Box childBox;
                childBox.width = node->m_box.width / 2.0;
                childBox.height = node->m_box.height / 2.0;
                childBox.depth = node->m_box.depth / 2.0;

                // 计算子节点中心
                // childIdx 的二进制位： bit0=x, bit1=y, bit2=z (0=-, 1=+)
                float dx = (childIdx & 1) ? 1.0f : -1.0f;
                float dy = (childIdx & 2) ? 1.0f : -1.0f;
                float dz = (childIdx & 4) ? 1.0f : -1.0f;

                childBox.translation = node->m_box.translation + Eigen::Vector3f(
                        dx * childBox.width * 0.5f,
                        dy * childBox.height * 0.5f,
                        dz * childBox.depth * 0.5f
                );

                // 创建新节点
                auto newChild = new OctreeNode(childBox, node->m_depth + 1, node);

                // 新节点初始化为叶子，分配 Block
                newChild->m_block = std::make_shared<CloudBlock>();

                // 继承父节点的属性状态 (如是否启用了颜色/法线)
                // 注意：这里需要确保新 Block 的 m_colors/m_normals 指针被初始化
                if (m_has_rgb) newChild->m_block->m_colors = std::make_unique<std::vector<RGB>>();
                if (m_has_normals) newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();

                if (color && !newChild->m_block->m_colors) {
                    newChild->m_block->m_colors = std::make_unique<std::vector<RGB>>();
                    // 同时更新 Cloud 的状态
                    m_has_rgb = true;
                }
                if (normal && !newChild->m_block->m_normals) {
                    newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();
                    m_has_normals = true;
                }

                // 注册到列表
                m_all_blocks.push_back(newChild->m_block);

                node->m_children[childIdx] = newChild;
            }

            // 递归
            return insertPointToOctree(node->m_children[childIdx], pt, color, normal);
        }
    }

    void Cloud::splitNode(OctreeNode* node)
    {
        if (!node->isLeaf()) return;

        // 获取原 Block 的数据所有权
        auto oldBlock = node->m_block;

        // 将当前节点标记为非叶子 (移除 Block 引用)
        node->m_block = nullptr;

        size_t n_points = oldBlock->size();

        // 准备随机采样器用于 LOD
        // 每 10 个点保留 1 个作为当前节点的 LOD 代理
        int lod_interval = 30;

        // 分发数据
        for (size_t i = 0; i < n_points; ++i) {
            const PointXYZ& pt = oldBlock->m_points[i];

            // --- LOD 处理 ---
            if (i % lod_interval == 0) {
                PointXYZRGB lod_pt;
                lod_pt.x = pt.x; lod_pt.y = pt.y; lod_pt.z = pt.z;
                if (oldBlock->m_colors) {
                    const auto& c = (*oldBlock->m_colors)[i];
                    lod_pt.r = c.r; lod_pt.g = c.g; lod_pt.b = c.b;
                } else {
                    lod_pt.r = 255; lod_pt.g = 255; lod_pt.b = 255;
                }
                node->m_lod_points.push_back(lod_pt);
            }

            // --- 子节点查找与创建 ---
            int childIdx = node->getChildIndex(pt);
            if (!node->m_children[childIdx]) {
                // 计算子节点包围盒
                Box childBox;
                childBox.width = node->m_box.width * 0.5f;
                childBox.height = node->m_box.height * 0.5f;
                childBox.depth = node->m_box.depth * 0.5f;

                float dx = (childIdx & 1) ? 1.0f : -1.0f;
                float dy = (childIdx & 2) ? 1.0f : -1.0f;
                float dz = (childIdx & 4) ? 1.0f : -1.0f;

                childBox.translation = node->m_box.translation + Eigen::Vector3f(
                        dx * childBox.width * 0.5f,
                        dy * childBox.height * 0.5f,
                        dz * childBox.depth * 0.5f
                );

                // new OctreeNode(box, depth, parent)
                auto newChild = new OctreeNode(childBox, node->m_depth + 1, node);
                newChild->m_block = std::make_shared<CloudBlock>();

                // 同步属性状态 (RGB/Normal)
                if (m_has_rgb) newChild->m_block->m_colors = std::make_unique<std::vector<RGB>>();
                if (m_has_normals) newChild->m_block->m_normals = std::make_unique<std::vector<CompressedNormal>>();

                // 同步标量场定义 (Key)
                if (!oldBlock->m_scalar_fields.isEmpty()) {
                    for(auto it = oldBlock->m_scalar_fields.begin(); it != oldBlock->m_scalar_fields.end(); ++it) {
                        newChild->m_block->registerScalarField(it.key());
                    }
                }

                // 注册到全局列表
                m_all_blocks.push_back(newChild->m_block);
                node->m_children[childIdx] = newChild;
            }

            // --- 数据移动 ---
            auto childBlock = node->m_children[childIdx]->m_block;

            // 基础属性
            childBlock->m_points.push_back(pt);

            if (oldBlock->m_colors && childBlock->m_colors) {
                childBlock->m_colors->push_back((*oldBlock->m_colors)[i]);
            } else if (childBlock->m_colors) {
                // 异常情况补白
                childBlock->m_colors->push_back(Color::White);
            }

            if (oldBlock->m_normals && childBlock->m_normals) {
                childBlock->m_normals->push_back((*oldBlock->m_normals)[i]);
            } else if (childBlock->m_normals) {
                childBlock->m_normals->push_back(CompressedNormal());
            }

            // 标量场 (高效批量处理)
            // 由于 registerScalarField 已经保证了 Key 存在，这里直接查找并 push_back
            if (!oldBlock->m_scalar_fields.isEmpty()) {
                // 遍历 oldBlock 的所有标量场
                auto it_old = oldBlock->m_scalar_fields.begin();
                auto it_child = childBlock->m_scalar_fields.begin();

                // 假设 map 的 key 顺序是一致的 (通常 QMap 是排序的)，可以同步迭代优化
                // 但为了安全，还是标准查找
                for (; it_old != oldBlock->m_scalar_fields.end(); ++it_old) {
                    float val = it_old.value()[i];
                    // 在 childBlock 中找到对应的 vector 并 push_back
                    // operator[] 会自动创建（如果不存在），但我们已经在创建节点时 register 了
                    childBlock->m_scalar_fields[it_old.key()].push_back(val);
                }
            }
        }

        // 清空旧块，释放内存
        oldBlock->clear();

        // 标记旧块为 dirty，虽然为空，但在某些渲染逻辑中可能需要刷新状态
        oldBlock->m_is_dirty = true;
    }

    void Cloud::addPoints(const std::vector<PointXYZ>& pts,
                          const std::vector<RGB>* colors,
                          const std::vector<CompressedNormal>* normals,
                          const QMap<QString, std::vector<float>>* scalars)
    {
        if (pts.empty()) return;

        // 懒初始化
        if (!m_octree_root) {
            // 计算包围盒
            PointXYZ min_pt, max_pt;
            getMinMax3D(pts, min_pt, max_pt);
            Eigen::Vector3f center(min_pt.x, min_pt.y, min_pt.z); // 大致以起点为中心

            // TODO 能不能动态设置包围盒
            Box box;
            box.width = 10000.0;  // 10 km
            box.height = 10000.0;
            box.depth = 10000.0;
            box.translation = center;
            initOctree(box);
        }

        // 预处理标量场：检查维度是否一致
        if (scalars) {
            for (auto it = scalars->begin(); it != scalars->end(); ++it) {
                if (it.value().size() != pts.size()) {
                    continue;
                }
            }
        }

        size_t n = pts.size();

        // 串行插入 (因为 insertPointToOctree 可能触发 split，涉及树结构修改，不适合并行)
        for (size_t i = 0; i < n; ++i) {
            const RGB* c = (colors && i < colors->size()) ? &(*colors)[i] : nullptr;
            const CompressedNormal* nm = (normals && i < normals->size()) ? &(*normals)[i] : nullptr;

            // 插入几何数据 (可能触发 split)
            CloudBlock* targetBlock = insertPointToOctree(m_octree_root.get(), pts[i], c, nm);

            // 补充标量数据 (如果有)
            if (scalars && targetBlock) {

                int block_size = targetBlock->size();
                size_t idx_in_block = targetBlock->size() - 1;

                for (auto it = scalars->begin(); it != scalars->end(); ++it) {
                    const QString& name = it.key();
                    float val = it.value()[i];

                    // 确保 Block 有这个字段 (自动注册)
                    targetBlock->registerScalarField(name);

                    // 写入数据
                    targetBlock->m_scalar_fields[name][idx_in_block] = val;
                }
            }
        }

        m_point_count += n;
        if (colors) m_color_modified = true;
    }

    void Cloud::addPoint(const ct::PointXYZRGBN &pt) {
        RGB color(pt.r, pt.g, pt.b);
        CompressedNormal normal;
        normal.set(Eigen::Vector3f(pt.normal_x, pt.normal_y, pt.normal_z));
        addPoint(PointXYZ(pt.x, pt.y, pt.z), &color, &normal);
    }

    // ===== 容量接口 =====
    size_t Cloud::size() const
    {
        return m_point_count;
    }

    bool Cloud::empty() const
    {
        return m_point_count == 0;
    }

    void Cloud::clear()
    {
        m_octree_root.reset(); // 释放整个树
        m_all_blocks.clear();  // 清空块列表
        m_point_count = 0;

        m_has_rgb = false;
        m_has_normals = false;

        // 清理缓存
        m_cached_xyz.reset();
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_render_cloud.reset();
    }

    void Cloud::enableColors()
    {
        if (m_has_rgb) return;

        // 遍历所有块，为每个块分配颜色内存
        for (auto& block : m_all_blocks) {
            if (!block->m_colors) {
                block->m_colors = std::make_unique<std::vector<RGB>>();
                // 如果块里已经有点了，需要补齐白色
                if (!block->m_points.empty()) {
                    block->m_colors->resize(block->size(), Color::White);
                }
            }
        }
        m_has_rgb = true;
        invalidateCache();
    }

    void Cloud::enableNormals()
    {
        if (m_has_normals) return;

        for (auto& block : m_all_blocks) {
            if (!block->m_normals) {
                block->m_normals = std::make_unique<std::vector<CompressedNormal>>();
                if (!block->m_points.empty()) {
                    block->m_normals->resize(block->size()); // 默认构造为 0
                }
            }
        }
        m_has_normals = true;
        invalidateCache();
    }

    void Cloud::disableColors()
    {
        for (auto& block : m_all_blocks) {
            block->m_colors.reset();
        }
        m_has_rgb = false;
        invalidateCache();
    }

    void Cloud::disableNormals()
    {
        for (auto& block : m_all_blocks) {
            block->m_normals.reset();
        }
        m_has_normals = false;
        invalidateCache();
    }

    // 清空缓存
    void Cloud::invalidateCache()
    {
        m_cache_valid = false;
        m_render_cache_valid = false;
        // 释放内存，防止占用过多
        m_cached_xyz.reset();
        m_cached_xyzrgb.reset();
        m_cached_xyzrgbn.reset();
        m_render_cloud.reset();
        // 标量场缓存也应该清空
        m_scalar_cache.clear();
    }

    pcl::PointCloud<PointXYZ>::Ptr Cloud::toPCL_XYZ() const
    {
        if (m_cache_valid && m_cached_xyz) return m_cached_xyz;

        m_cached_xyz = std::make_shared<pcl::PointCloud<PointXYZ>>();
        m_cached_xyz->reserve(m_point_count);

        // 线性遍历所有块进行拼接
        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;
            // 批量插入
            m_cached_xyz->insert(m_cached_xyz->end(), block->m_points.begin(), block->m_points.end());
        }

        return m_cached_xyz;
    }

    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::toPCL_XYZRGB() const
    {
        if (m_cache_valid && m_cached_xyzrgb) return m_cached_xyzrgb;

        m_cached_xyzrgb = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
        m_cached_xyzrgb->resize(m_point_count);

        size_t global_idx = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* cols = block->m_colors.get();

            // 块内并行拷贝
#pragma omp parallel for if(n > 10000)
            for (int i = 0; i < (int)n; ++i) {
                auto& dst = m_cached_xyzrgb->points[global_idx + i];
                const auto& src_pt = pts[i];

                dst.x = src_pt.x; dst.y = src_pt.y; dst.z = src_pt.z;

                if (cols) {
                    const auto& c = (*cols)[i];
                    dst.r = c.r; dst.g = c.g; dst.b = c.b;
                } else {
                    dst.r = 255; dst.g = 255; dst.b = 255;
                }
            }
            global_idx += n;
        }

        m_cache_valid = true;
        return m_cached_xyzrgb;
    }

    pcl::PointCloud<PointXYZRGBN>::Ptr Cloud::toPCL_XYZRGBN() const
    {
        if (m_cache_valid && m_cached_xyzrgbn) return m_cached_xyzrgbn;

        m_cached_xyzrgbn = std::make_shared<pcl::PointCloud<PointXYZRGBN>>();
        m_cached_xyzrgbn->resize(m_point_count);

        size_t global_idx = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* cols = block->m_colors.get();
            const auto* norms = block->m_normals.get();

#pragma omp parallel for if(n > 10000)
            for (int i = 0; (int)i < n; ++i) {
                auto& dst = m_cached_xyzrgbn->points[global_idx + i];
                const auto& src_pt = pts[i];

                dst.x = src_pt.x; dst.y = src_pt.y; dst.z = src_pt.z;

                if (cols) {
                    const auto& c = (*cols)[i];
                    dst.r = c.r; dst.g = c.g; dst.b = c.b;
                } else {
                    dst.r = 255; dst.g = 255; dst.b = 255;
                }

                if (norms) {
                    Eigen::Vector3f n_vec = (*norms)[i].get();
                    dst.normal_x = n_vec.x(); dst.normal_y = n_vec.y(); dst.normal_z = n_vec.z();
                } else {
                    dst.normal_x = 0; dst.normal_y = 0; dst.normal_z = 0;
                }
            }
            global_idx += n;
        }

        m_cache_valid = true;
        return m_cached_xyzrgbn;
    }

    // =========================================================
    // TODO 渲染接口 (临时方案：依然返回拼接点云)
    // =========================================================

    pcl::PointCloud<PointXYZRGB>::Ptr Cloud::getRenderCloud() const
    {
        // 阶段一：我们还没有实现自定义 Mapper，所以 View 层依然需要一个完整的 PCL 点云
        // 这里直接复用 toPCL_XYZRGB
        if (m_render_cache_valid && m_render_cloud) return m_render_cloud;

        m_render_cloud = toPCL_XYZRGB();
        m_render_cache_valid = true;
        return m_render_cloud;
    }

    void Cloud::invalidateRenderCache()
    {
        m_render_cloud.reset();
        m_render_cache_valid = false;
    }

    void Cloud::addScalarField(const QString& name, const std::vector<float>& data)
    {
        if (data.size() != m_point_count) return;

        // 需要将这一个大 vector 切分到各个 Block 中
        size_t global_offset = 0;

        for (auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t n = block->size();
            // 获取 Block 内部对应字段的引用 (会自动创建)
            block->registerScalarField(name);
            std::vector<float>& block_data = block->m_scalar_fields[name];

            // 拷贝数据片段
            // 这里的 data.begin() + global_offset 是迭代器偏移
            std::copy(data.begin() + global_offset,
                      data.begin() + global_offset + n,
                      block_data.begin());

            global_offset += n;
        }

        // 清理一下导出缓存，因为数据变了
        m_scalar_cache.remove(name);
    }

    bool Cloud::removeScalarField(const QString& name)
    {
        bool found = false;
        for (auto& block : m_all_blocks) {
            if (block->m_scalar_fields.remove(name) > 0) {
                found = true;
            }
        }
        m_scalar_cache.remove(name);
        return found;
    }

    void Cloud::clearScalarFields()
    {
        for (auto& block : m_all_blocks) {
            block->m_scalar_fields.clear();
        }
        m_scalar_cache.clear();
    }

    bool Cloud::hasScalarField(const QString& name) const
    {
        // 只要第一个非空 Block 有这个字段，就认为有
        for (const auto& block : m_all_blocks) {
            if (!block->empty()) {
                return block->m_scalar_fields.contains(name);
            }
        }
        return false;
    }

    QStringList Cloud::getScalarFieldNames() const
    {
        // 从第一个非空 Block 获取 keys
        for (const auto& block : m_all_blocks) {
            if (!block->empty()) {
                return block->m_scalar_fields.keys();
            }
        }
        return QStringList();
    }

    const std::vector<float>* Cloud::getScalarField(const QString& name) const
    {
        // 检查缓存中是否已有拼接好的数据
        if (m_scalar_cache.contains(name)) {
            // 校验一下大小是否匹配 (防止数据更新后缓存未清理的极端情况)
            if (m_scalar_cache[name].size() == m_point_count) {
                return &m_scalar_cache[name];
            }
        }

        // 如果没有，或者缓存无效，执行拼接
        if (!hasScalarField(name)) return nullptr;

        std::vector<float>& full_data = m_scalar_cache[name]; // 创建/覆盖缓存条目
        full_data.resize(m_point_count);

        size_t global_offset = 0;
        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            // 查找 Block 内的字段
            auto it = block->m_scalar_fields.find(name);
            if (it != block->m_scalar_fields.end()) {
                const std::vector<float>& src = it.value();
                std::copy(src.begin(), src.end(), full_data.begin() + global_offset);
            } else {
                // 如果某个 Block 缺失该字段 (异常情况)，填充 0
                std::fill_n(full_data.begin() + global_offset, block->size(), 0.0f);
            }
            global_offset += block->size();
        }

        return &full_data;
    }

    void Cloud::setCloudColor(const RGB& rgb)
    {
        if (!m_has_rgb) enableColors();
        backupColors(); // 自动备份

#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            if (block->empty()) continue;

            // 直接批量赋值，比循环快
            std::fill(block->m_colors->begin(), block->m_colors->end(), rgb);

            block->m_is_dirty = true; // 确保标记为脏
            block->m_vtk_polydata.reset(); // 释放旧缓存
        }
        if (m_octree_root) {
            updateLODColorRecursive(m_octree_root.get(), rgb);
        }

        m_color_modified = true;
        invalidateRenderCache(); // 通知视图更新
    }

    void Cloud::setCloudColor(const QString& axis)
    {
        if (empty()) return;
        if (!m_has_rgb) enableColors();
        backupColors();

        float min_v = 0.0f, max_v = 0.0f;
        if (axis == "x") { min_v = m_min.x; max_v = m_max.x; }
        else if (axis == "y") { min_v = m_min.y; max_v = m_max.y; }
        else if (axis == "z") { min_v = m_min.z; max_v = m_max.z; }
        else return;

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;

        // HSV 转 RGB lambda (保持不变)
        auto hsv2rgb = [](float h, float s, float v, uint8_t& r, uint8_t& g, uint8_t& b) {
            // ... (同原代码，省略以节省篇幅) ...
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
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            if (block->empty()) continue;

            size_t n = block->size();
            for (size_t i = 0; i < n; ++i) {
                const auto& pt = block->m_points[i];
                float val = (axis == "x") ? pt.x : (axis == "y") ? pt.y : pt.z;

                float norm = (val - min_v) / range;
                float hue = (1.0f - norm) * 0.66f;

                hsv2rgb(hue, 1.0f, 1.0f, (*block->m_colors)[i].r, (*block->m_colors)[i].g, (*block->m_colors)[i].b);
            }
        }

        m_color_modified = true;
        invalidateRenderCache();
    }

    void Cloud::updateLODColorRecursive(OctreeNode* node, const RGB& rgb)
    {
        if (!node) return;

        // 更新当前节点的 LOD 数据颜色
        if (!node->m_lod_points.empty()) {
            for (auto& p : node->m_lod_points) {
                p.r = rgb.r;
                p.g = rgb.g;
                p.b = rgb.b;
            }
            // 标记 LOD 脏，强制 VTK 重新上传
            node->m_lod_dirty = true;
            node->m_vtk_lod_polydata.reset();
        }

        // 递归子节点
        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                updateLODColorRecursive(node->m_children[i], rgb);
            }
        }
    }

    void Cloud::updateColorByField(const QString& field_name)
    {
        if (!hasScalarField(field_name)) return;
        if (s_jet_lut.empty()) initColorTable();
        if (!m_has_rgb) enableColors();
        backupColors();

        // 1. 计算全局 Min/Max
        // 需要遍历所有 Block 才能得到全局极值
        float min_v = FLT_MAX;
        float max_v = -FLT_MAX;

#pragma omp parallel
        {
            float local_min = FLT_MAX;
            float local_max = -FLT_MAX;

#pragma omp for
            for (int k = 0; k < m_all_blocks.size(); ++k) {
                auto& block = m_all_blocks[k];
                if (block->empty() || !block->m_scalar_fields.contains(field_name)) continue;

                const std::vector<float>& data = block->m_scalar_fields[field_name];
                for (float v : data) {
                    if (v < local_min) local_min = v;
                    if (v > local_max) local_max = v;
                }
            }

#pragma omp critical
            {
                if (local_min < min_v) min_v = local_min;
                if (local_max > max_v) max_v = local_max;
            }
        }

        float range = max_v - min_v;
        if (range < 1e-6f) range = 1.0f;
        const float* pLut = s_jet_lut.data();
        int lut_size = 1024;

        // 2. 应用颜色
#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];
            if (block->empty() || !block->m_scalar_fields.contains(field_name)) continue;

            const std::vector<float>& data = block->m_scalar_fields[field_name];
            size_t n = block->size();

            for (size_t i = 0; i < n; ++i) {
                float norm = (data[i] - min_v) / range;
                if (norm < 0.0f) norm = 0.0f;
                if (norm > 1.0f) norm = 1.0f;

                int idx = static_cast<int>(norm * (lut_size - 1));
                float lutVal = pLut[idx];
                uint32_t packed = *reinterpret_cast<const uint32_t*>(&lutVal);

                (*block->m_colors)[i].r = (packed >> 16) & 0xFF;
                (*block->m_colors)[i].g = (packed >> 8) & 0xFF;
                (*block->m_colors)[i].b = packed & 0xFF;
            }
        }

        m_color_modified = true;
        invalidateRenderCache();
    }

    void Cloud::update()
    {
        if (empty() || !m_octree_root) {
            m_min = m_max = PointXYZ(0,0,0);
            return;
        }

        PointXYZ real_min(FLT_MAX, FLT_MAX, FLT_MAX);
        PointXYZ real_max(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        size_t total = 0;

        updateRecursive(m_octree_root.get(), real_min, real_max, total);

        m_min = real_min;
        m_max = real_max;
        m_point_count = total; // 修正总点数

        // 更新中心点
        Eigen::Vector3f center = 0.5f * (m_min.getVector3fMap() + m_max.getVector3fMap());
        m_box.pose.translation() = center;

        // 更新类型字符串
        if (m_has_normals && m_has_rgb) m_type = CLOUD_TYPE_XYZRGBN;
        else if (m_has_normals) m_type = CLOUD_TYPE_XYZN;
        else if (m_has_rgb) m_type = CLOUD_TYPE_XYZRGB;
        else m_type = CLOUD_TYPE_XYZ;
    }

    void Cloud::updateRecursive(OctreeNode* node, PointXYZ& min_pt, PointXYZ& max_pt, size_t& count)
    {
        if (!node) return;

        if (node->isLeaf()) {
            // 叶子节点：统计 Block 数据
            auto block = node->m_block;
            if (!block || block->empty()) return;

            count += block->size();

            // 这里简单串行处理
            for (const auto& p : block->m_points) {
                if (p.x < min_pt.x) min_pt.x = p.x;
                if (p.x > max_pt.x) max_pt.x = p.x;
                if (p.y < min_pt.y) min_pt.y = p.y;
                if (p.y > max_pt.y) max_pt.y = p.y;
                if (p.z < min_pt.z) min_pt.z = p.z;
                if (p.z > max_pt.z) max_pt.z = p.z;
            }

            // 顺便更新 Block 的 AABB（如果需要的话）
            // block->updateBoundingBox();
        }
        else {
            // 内部节点：递归
            for (int i = 0; i < 8; ++i) {
                updateRecursive(node->m_children[i], min_pt, max_pt, count);
            }
        }
    }

    void Cloud::backupColors()
    {
#pragma omp parallel for
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];

            if (block->empty() || !block->m_colors) continue;

            block->m_backup_colors = std::make_unique<std::vector<RGB>>(*block->m_colors);
        }
    }

    void Cloud::restoreColors()
    {
        bool any_restored = false;

#pragma omp parallel for reduction(|:any_restored)
        for (int k = 0; k < m_all_blocks.size(); ++k) {
            auto& block = m_all_blocks[k];

            // 只有当存在备份时才恢复
            if (block->m_backup_colors) {
                // 如果当前 m_colors 被释放了（比如调用过 disableColors），需要重新分配
                if (!block->m_colors) {
                    block->m_colors = std::make_unique<std::vector<RGB>>();
                }

                *block->m_colors = *block->m_backup_colors;
                if (!block->m_colors->empty()) any_restored = true;

                // CloudCompare 的逻辑通常是保留备份直到显式清除或覆盖
                // 这里我们保留备份，允许用户反复切换
            }
        }

        if (any_restored) {
            m_has_rgb = true; // 确保 Cloud 状态正确
            m_color_modified = false; // 颜色已恢复到原始状态（或上一次备份的状态）
            invalidateRenderCache();
        }
    }

    Cloud::Ptr Cloud::clone() const
    {
        Cloud::Ptr result(new Cloud);
        result->copyFrom(*this);
        return result;
    }

    void Cloud::copyFrom(const Cloud& other)
    {
        // 清空当前对象
        this->clear();

        // 基础属性拷贝
        this->m_id = other.m_id + "_clone";
        this->m_info = other.m_info;
        this->m_box = other.m_box;
        this->m_box_rgb = other.m_box_rgb;
        this->m_normals_rgb = other.m_normals_rgb;
        this->m_point_size = other.m_point_size;
        this->m_opacity = other.m_opacity;
        this->m_resolution = other.m_resolution;
        this->m_min = other.m_min;
        this->m_max = other.m_max;
        this->m_global_shift = other.m_global_shift;
        this->m_has_rgb = other.m_has_rgb;
        this->m_has_normals = other.m_has_normals;
        this->m_type = other.m_type;
        this->m_max_depth = other.m_max_depth;

        // 递归深拷贝八叉树结构
        // 这一步会自动重建 m_all_blocks 列表
        if (other.m_octree_root) {
            this->m_octree_root = cloneOctreeRecursive(other.m_octree_root.get(), nullptr);
        }

        // 复制统计数据
        this->m_point_count = other.m_point_count;
        this->m_color_modified = other.m_color_modified;
    }

    std::unique_ptr<OctreeNode> Cloud::cloneOctreeRecursive(const OctreeNode* src_node, OctreeNode* parent)
    {
        if (!src_node) return nullptr;

        // 克隆当前节点 (Metadata)
        auto new_node = std::make_unique<OctreeNode>(src_node->m_box, src_node->m_depth, parent);
        new_node->m_total_points_in_node = src_node->m_total_points_in_node;

        // 如果有 LOD 数据，也一并拷贝
        new_node->m_lod_points = src_node->m_lod_points;

        // 如果是叶子节点，深拷贝 Block 数据
        if (src_node->isLeaf() && src_node->m_block) {
            new_node->m_block = std::make_shared<CloudBlock>();

            const auto& src_block = src_node->m_block;
            auto& dst_block = new_node->m_block;

            // --- 核心数据拷贝 ---
            dst_block->m_points = src_block->m_points;
            dst_block->m_box = src_block->m_box;

            // 颜色 (深拷贝 vector)
            if (src_block->m_colors)
                dst_block->m_colors = std::make_unique<std::vector<RGB>>(*src_block->m_colors);

            // 法线
            if (src_block->m_normals)
                dst_block->m_normals = std::make_unique<std::vector<CompressedNormal>>(*src_block->m_normals);

            // 标量场 (QMap 具有值语义，直接赋值会触发深拷贝)
            dst_block->m_scalar_fields = src_block->m_scalar_fields;

            // 备份颜色
            if (src_block->m_backup_colors)
                dst_block->m_backup_colors = std::make_unique<std::vector<RGB>>(*src_block->m_backup_colors);

            // 渲染状态
            dst_block->m_is_visible = src_block->m_is_visible;
            dst_block->m_is_dirty = true; // 新数据，标记为脏，以便上传 GPU

            // 将新 Block 注册到 Cloud 的扁平化列表中
            this->m_all_blocks.push_back(dst_block);
        }

        // 递归克隆子节点
        for (int i = 0; i < 8; ++i) {
            if (src_node->m_children[i]) {
                // 递归调用，并将返回的 unique_ptr 所有权释放给裸指针数组 m_children
                // 注意：这里传入 new_node.get() 作为 parent
                new_node->m_children[i] = cloneOctreeRecursive(src_node->m_children[i], new_node.get()).release();
            }
        }

        return new_node;
    }

    void Cloud::append(const Cloud& other)
    {
        if (other.empty()) return;

        if (other.hasColors()) this->enableColors();
        if (other.hasNormals()) this->enableNormals();

        // 遍历源对象的所有 Block
        for (const auto& block : other.m_all_blocks) {
            if (block->empty()) continue;

            // 基础属性
            const std::vector<RGB>* c = (block->m_colors) ? block->m_colors.get() : nullptr;
            const std::vector<CompressedNormal>* n = (block->m_normals) ? block->m_normals.get() : nullptr;

            // 标量场 (QMap 指针)
            // addPoints 接受的是 const QMap<QString, vector<float>>*
            // Block 中存储的正是这个类型，直接取地址即可
            const QMap<QString, std::vector<float>>* s =
                    (!block->m_scalar_fields.isEmpty()) ? &block->m_scalar_fields : nullptr;

            // 批量插入
            this->addPoints(block->m_points, c, n, s);
        }

        update();

        // 缓存失效
        invalidateCache();
        invalidateRenderCache();
    }

    Cloud& Cloud::operator+=(const Cloud& rhs)
    {
        append(rhs);
        return *this;
    }

    Cloud::Ptr Cloud::fromPCL_XYZRGBN(const pcl::PointCloud<PointXYZRGBN>& pcl_cloud)
    {
        Cloud::Ptr cloud(new Cloud);
        if (pcl_cloud.empty()) return cloud;

        // 计算包围盒以初始化八叉树
        PointXYZRGBN min_pt, max_pt;
        pcl::getMinMax3D<PointXYZRGBN>(pcl_cloud, min_pt, max_pt);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(min_pt.x + box.width/2, min_pt.y + box.height/2, min_pt.z + box.depth/2);

        cloud->initOctree(box);
        cloud->enableColors();
        cloud->enableNormals();

        // 转换数据格式以便批量添加
        // 为了避免一次性拷贝所有数据造成内存峰值，我们可以分批次添加
        size_t n = pcl_cloud.size();
        size_t batch_size = 50000;

        std::vector<PointXYZ> pts; pts.reserve(batch_size);
        std::vector<RGB> colors; colors.reserve(batch_size);
        std::vector<CompressedNormal> normals; normals.reserve(batch_size);

        for (size_t i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            pts.emplace_back(src.x, src.y, src.z);
            colors.emplace_back(src.r, src.g, src.b);

            CompressedNormal cn;
            cn.set(Eigen::Vector3f(src.normal_x, src.normal_y, src.normal_z));
            normals.push_back(cn);

            if (pts.size() >= batch_size) {
                cloud->addPoints(pts, &colors, &normals);
                pts.clear(); colors.clear(); normals.clear();
            }
        }

        // 添加剩余的
        if (!pts.empty()) {
            cloud->addPoints(pts, &colors, &normals);
        }

        cloud->update();
        return cloud;
    }

    Cloud::Ptr Cloud::fromPCL_XYZRGB(const pcl::PointCloud<PointXYZRGB> &pcl_cloud) {
        Cloud::Ptr cloud(new Cloud);
        if (pcl_cloud.empty()) return cloud;

        // 计算包围盒以初始化八叉树
        PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D<PointXYZRGB>(pcl_cloud, min_pt, max_pt);

        Box box;
        box.width = max_pt.x - min_pt.x;
        box.height = max_pt.y - min_pt.y;
        box.depth = max_pt.z - min_pt.z;
        if (box.width < 1e-6) box.width = 1.0;
        if (box.height < 1e-6) box.height = 1.0;
        if (box.depth < 1e-6) box.depth = 1.0;
        box.translation = Eigen::Vector3f(min_pt.x + box.width/2, min_pt.y + box.height/2, min_pt.z + box.depth/2);

        cloud->initOctree(box);
        cloud->enableColors();
        cloud->enableNormals();

        // 转换数据格式以便批量添加
        // 为了避免一次性拷贝所有数据造成内存峰值，我们可以分批次添加
        size_t n = pcl_cloud.size();
        size_t batch_size = 50000;

        std::vector<PointXYZ> pts; pts.reserve(batch_size);
        std::vector<RGB> colors; colors.reserve(batch_size);


        for (size_t i = 0; i < n; ++i) {
            const auto& src = pcl_cloud.points[i];

            pts.emplace_back(src.x, src.y, src.z);
            colors.emplace_back(src.r, src.g, src.b);


            if (pts.size() >= batch_size) {
                cloud->addPoints(pts, &colors);
                pts.clear(); colors.clear();
            }
        }

        // 添加剩余的
        if (!pts.empty()) {
            cloud->addPoints(pts, &colors);
        }

        cloud->update();
        return cloud;
    }

    void Cloud::removeInvalidPoints()
    {
        // 遍历所有 Block，移除 NaN
        for (auto& block : m_all_blocks) {
            if (block->empty()) continue;

            // 简单的 remove_if + erase 逻辑
            // 需要同步处理 points, colors, normals, scalars
            // 这在 SOA 结构下比较麻烦，需要手写压缩逻辑

            size_t write_idx = 0;
            size_t read_idx = 0;
            size_t n = block->size();

            for (; read_idx < n; ++read_idx) {
                const auto& p = block->m_points[read_idx];
                if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                    if (write_idx != read_idx) {
                        block->m_points[write_idx] = p;
                        if (block->m_colors) (*block->m_colors)[write_idx] = (*block->m_colors)[read_idx];
                        if (block->m_normals) (*block->m_normals)[write_idx] = (*block->m_normals)[read_idx];
                        // scalars...
                    }
                    write_idx++;
                }
            }

            // Resize
            block->m_points.resize(write_idx);
            if (block->m_colors) block->m_colors->resize(write_idx);
            if (block->m_normals) block->m_normals->resize(write_idx);
        }

        invalidateCache();
        update();
    }

    // 初始化颜色映射表 (Jet Colormap)
    void Cloud::initColorTable()
    {
        // 静态成员只需初始化一次
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

    // TODO 获取指定全局索引点的颜色 (用于保存文件)
    // 注意：这是一个 O(N) 操作，如果在循环中对每个点都调用这个函数，保存大文件会非常慢！
    // 强烈建议：在保存逻辑中直接遍历 Block，而不是遍历全局索引。
    // 但为了保持接口兼容性，我们先实现它。
    std::uint32_t Cloud::getPointColorForSave(size_t index) const
    {
        // 快速检查
        if (!m_has_rgb) return (255<<16) | (255<<8) | 255; // White

        // 线性查找 Block (性能瓶颈)
        // 优化：保存算法应该重构为遍历 Block
        size_t current_offset = 0;

        for (const auto& block : m_all_blocks) {
            if (block->empty()) continue;

            size_t block_size = block->size();

            // 检查索引是否在当前 Block 内
            if (index < current_offset + block_size) {
                size_t local_idx = index - current_offset;

                if (block->m_colors) {
                    const RGB& c = (*block->m_colors)[local_idx];
                    return (c.r << 16) | (c.g << 8) | c.b;
                } else {
                    return (255<<16) | (255<<8) | 255;
                }
            }

            current_offset += block_size;
        }

        // 索引越界
        return (255<<16) | (255<<8) | 255;
    }

} // namespace ct
