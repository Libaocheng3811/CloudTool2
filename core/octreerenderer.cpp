//
// Created by LBC on 2026/1/28.
//

#include "octreerenderer.h"

#include <vtkRenderer.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkCamera.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkPlane.h>

namespace ct{
    OctreeRenderer::OctreeRenderer(Cloud::Ptr cloud, vtkRenderer* renderer)
            : m_cloud(cloud), m_vtk_renderer(renderer)
    {
        // 初始化 Actor 池
        m_actor_pool.resize(MAX_ACTORS);

        for (int i = 0; i < MAX_ACTORS; ++i) {
            m_actor_pool[i].mapper = vtkPolyDataMapper::New();

            m_actor_pool[i].mapper->SetScalarModeToUsePointData();
            m_actor_pool[i].mapper->SetColorModeToDirectScalars();

            m_actor_pool[i].actor = vtkActor::New();
            m_actor_pool[i].actor->SetMapper(m_actor_pool[i].mapper);

            // 初始隐藏
            m_actor_pool[i].actor->SetVisibility(0);

            // 默认属性 (点大小等)
            m_actor_pool[i].actor->GetProperty()->SetPointSize(cloud->pointSize());
            m_actor_pool[i].actor->GetProperty()->SetOpacity(cloud->opacity());

            // 加入到 Renderer 中
            // 注意：一次性加入 500 个隐藏的 Actor 对 VTK 性能影响很小
            m_vtk_renderer->AddActor(m_actor_pool[i].actor);
        }
    }

    OctreeRenderer::~OctreeRenderer()
    {
        // 移除并删除 Actor
        if (m_vtk_renderer) {
            for (auto& item : m_actor_pool) {
                m_vtk_renderer->RemoveActor(item.actor);
                item.actor->Delete();
                item.mapper->Delete();
            }
        }
    }

    void OctreeRenderer::update()
    {
        if (!m_cloud || !m_cloud->getOctreeRoot()) return;

        if (!m_visible) {
            // 如果设置为不可见，强制隐藏所有当前活跃的 Actor
            for (int i = 0; i < MAX_ACTORS; ++i) {
                // 只要 Actor 是可见的，就隐藏它
                if (m_actor_pool[i].actor->GetVisibility() != 0) {
                    m_actor_pool[i].actor->SetVisibility(0);
                }
                // 重置状态，防止逻辑残留
                m_actor_pool[i].is_active = false;
                m_actor_pool[i].block.reset();
                m_actor_pool[i].lodNode = nullptr;
            }

            // 清空映射表和计数
            m_block_to_actor_index.clear();
            m_used_actor_count = 0;

            return; // 直接退出，不执行后续渲染逻辑
        }

        // 获取视锥体平面
        vtkCamera* cam = m_vtk_renderer->GetActiveCamera();
        double planes[24]; // 6 planes * 4 coeffs
        cam->GetFrustumPlanes(m_vtk_renderer->GetTiledAspectRatio(), planes);

        double camPosDouble[3];
        cam->GetPosition(camPosDouble);
        Eigen::Vector3f camPos(camPosDouble[0], camPosDouble[1], camPosDouble[2]);

        //  遍历八叉树，收集可见块
        std::vector<CloudBlock::Ptr> visibleBlocks;
        std::vector<OctreeNode*> visibleLODs;
        visibleBlocks.reserve(MAX_ACTORS);

        traverseOctree(m_cloud->getOctreeRoot(), planes, camPos, visibleBlocks, visibleLODs);

        // 分配 Actor 并渲染
        assignActors(visibleBlocks, visibleLODs);

        // 同步全局属性 (如点大小改变了)
        // todo 优化：可以加个 dirty flag，只在改变时设置
        for (int i = 0; i < m_used_actor_count; ++i) {
            m_actor_pool[i].actor->GetProperty()->SetPointSize(m_cloud->pointSize());
            m_actor_pool[i].actor->GetProperty()->SetOpacity(m_cloud->opacity());
        }
    }

    bool OctreeRenderer::isBoxInFrustum(const Box& box, const double* planes)
    {
        // 检查 Box 的 8 个角点，而不是只检查中心
        double bounds[6] = {
                box.translation.x() - box.width/2, box.translation.x() + box.width/2,
                box.translation.y() - box.height/2, box.translation.y() + box.height/2,
                box.translation.z() - box.depth/2, box.translation.z() + box.depth/2
        };

        double corners[8][3];
        corners[0][0] = bounds[0]; corners[0][1] = bounds[2]; corners[0][2] = bounds[4];
        corners[1][0] = bounds[1]; corners[1][1] = bounds[2]; corners[1][2] = bounds[4];
        corners[2][0] = bounds[0]; corners[2][1] = bounds[3]; corners[2][2] = bounds[4];
        corners[3][0] = bounds[1]; corners[3][1] = bounds[3]; corners[3][2] = bounds[4];
        corners[4][0] = bounds[0]; corners[4][1] = bounds[2]; corners[4][2] = bounds[5];
        corners[5][0] = bounds[1]; corners[5][1] = bounds[2]; corners[5][2] = bounds[5];
        corners[6][0] = bounds[0]; corners[6][1] = bounds[3]; corners[6][2] = bounds[5];
        corners[7][0] = bounds[1]; corners[7][1] = bounds[3]; corners[7][2] = bounds[5];

        // 对所有 6 个平面检查
        for (int i = 0; i < 6; ++i) {
            const double* plane = planes + i * 4;
            bool allOutside = true;

            for (int j = 0; j < 8; ++j) {
                double dist = plane[0] * corners[j][0] +
                              plane[1] * corners[j][1] +
                              plane[2] * corners[j][2] +
                              plane[3];
                if (dist > -1e-6) {  // 只要有一个点在平面内侧（或非常接近）
                    allOutside = false;
                    break;
                }
            }

            if (allOutside) return false; // 这个平面的所有点都在外侧
        }

        return true;
    }

    void OctreeRenderer::traverseOctree(OctreeNode* node, const double* planes,
                                        const Eigen::Vector3f& camPos,
                                        std::vector<CloudBlock::Ptr>& outVisibleBlocks,
                                        std::vector<OctreeNode*>& outVisibleLODs)
    {
        if (!node) return;

        // 视锥体剔除
        if (!isBoxInFrustum(node->m_box, planes)) return;

        // LOD 判断
        // 计算节点到相机的距离
        float dist = (node->m_box.translation - camPos).norm();
        float size = std::max({node->m_box.width, node->m_box.height, node->m_box.depth});

        // 投影大小估算 (size / distance)
        // 如果投影太小，且不是叶子节点，显示 LOD
        bool showLOD = false;
        if (dist > 1e-6) {
            float projected = size / dist;
            if (projected < m_lod_threshold && !node->isLeaf()) {
                showLOD = true;
            }
        }

        if (showLOD) {
            // 如果节点有 LOD 数据，添加到列表并停止递归
            if (!node->m_lod_points.empty()) {
                outVisibleLODs.push_back(node);
                return;
            }
            // 如果没有 LOD 数据，继续递归子节点显示详细内容
        }

        // 如果是叶子节点，显示 Block
        if (node->isLeaf()) {
            if (node->m_block && !node->m_block->empty()) {
                outVisibleBlocks.push_back(node->m_block);
            }
            return;
        }

        // 递归子节点
        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                traverseOctree(node->m_children[i], planes, camPos, outVisibleBlocks, outVisibleLODs);
            }
        }
    }

    void OctreeRenderer::assignActors(const std::vector<CloudBlock::Ptr>& blocks,
                                      const std::vector<OctreeNode*>& lods)
    {
        // 如果不可见，直接隐藏所有 Actor 并返回
        if (!m_visible) {
            for (auto& item : m_actor_pool) {
                item.actor->SetVisibility(0);
                item.is_active = false;
            }
            m_used_actor_count = 0;
            return;
        }

        // 1. 重置所有 Actor 的活跃状态
        for (auto& item : m_actor_pool) {
            item.is_active = false;
        }

        // 辅助 lambda：处理单个 Block/Node 的分配
        auto process_node = [&](void* key, CloudBlock::Ptr blk, OctreeNode* node) {
            // 如果数据脏了，先更新数据 (这一步保留原逻辑)
            if (blk && (blk->m_is_dirty || !blk->m_vtk_polydata)) {
                updateBlockPolyData(blk.get());
                blk->m_is_dirty = false;
            }
            if (node && (node->m_lod_dirty || !node->m_vtk_lod_polydata)) {
                updateLODPolyData(node);
                node->m_lod_dirty = false;
            }

            int assigned_idx = -1;

            // A. 检查缓存：该节点是否已经分配了 Actor？
            auto it = m_block_to_actor_index.find(key);
            if (it != m_block_to_actor_index.end()) {
                assigned_idx = it->second;
                // 确保这个 Actor 没有被别人抢占（防止极端的逻辑错误）
                if (m_actor_pool[assigned_idx].is_active) {
                    assigned_idx = -1; // 冲突了，重新分配
                }
            }

            // B. 如果没有分配，或者之前的 Actor 被占用了，找一个新的空闲 Actor
            if (assigned_idx == -1) {
                // 简单的线性搜索空闲位置
                // (为了性能，可以维护一个空闲链表，但这里 500 个循环很快)
                for (int i = 0; i < MAX_ACTORS; ++i) {
                    // 找一个这一帧还没被标记为 active 的，
                    // 且最好是上一帧也没显示东西的（避免频繁切换，这里简化为找非 active）
                    // 更好的策略是优先找 m_block_to_actor_index 中不存在的索引
                    if (!m_actor_pool[i].is_active) {
                        // 如果这个位置之前属于别的 Block，从映射表中移除旧的
                        void* old_key = (m_actor_pool[i].block) ? (void*)m_actor_pool[i].block.get() : (void*)m_actor_pool[i].lodNode;
                        if(old_key) m_block_to_actor_index.erase(old_key);

                        assigned_idx = i;
                        break;
                    }
                }
            }

            // C. 如果找到了 Actor，进行绑定
            if (assigned_idx != -1) {
                RenderItem& item = m_actor_pool[assigned_idx];
                item.is_active = true;
                item.block = blk;
                item.lodNode = node;

                // 更新映射表
                m_block_to_actor_index[key] = assigned_idx;

                // 设置 VTK 数据
                auto poly = blk ? std::static_pointer_cast<vtkPolyData>(blk->m_vtk_polydata)
                                : std::static_pointer_cast<vtkPolyData>(node->m_vtk_lod_polydata);

                // 【核心优化】：只有当 Input 真的改变时才调用 SetInputData
                if (item.mapper->GetInput() != poly.get()) {
                    item.mapper->SetInputData(poly.get());
                }

                // 确保可见
                if (item.actor->GetVisibility() == 0) {
                    item.actor->SetVisibility(1);
                }
            }
        };

        // 2. 处理 Blocks
        for (const auto& block : blocks) {
            process_node((void*)block.get(), block, nullptr);
        }

        // 3. 处理 LODs
        for (const auto& node : lods) {
            process_node((void*)node, nullptr, node);
        }

        // 4. 清理未使用的 Actor
        m_used_actor_count = 0;
        for (int i = 0; i < MAX_ACTORS; ++i) {
            if (!m_actor_pool[i].is_active) {
                if (m_actor_pool[i].actor->GetVisibility() != 0) {
                    m_actor_pool[i].actor->SetVisibility(0);
                    // 释放映射
                    void* key = (m_actor_pool[i].block) ? (void*)m_actor_pool[i].block.get() : (void*)m_actor_pool[i].lodNode;
                    if(key) m_block_to_actor_index.erase(key);

                    m_actor_pool[i].block.reset();
                    m_actor_pool[i].lodNode = nullptr;
                }
            } else {
                m_used_actor_count++;
            }
        }
    }

    void OctreeRenderer::updateBlockPolyData(CloudBlock* block)
    {
        // 将 vector 转换为 vtkPolyData
        auto poly = vtkSmartPointer<vtkPolyData>::New();
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto cells = vtkSmartPointer<vtkCellArray>::New();

        size_t n = block->size();

        // 这里可以使用 float 数组直接拷贝，甚至 zero-copy (vtkFloatArray::SetArray)
        // SetArray 需要保证 vector 在 polyData 生命周期内不销毁/扩容
        // 鉴于 CloudBlock 的 vector 是稳定的，可以使用 zero-copy 优化！
        // 但为了安全起见，第一版先用 deep copy。

        points->SetNumberOfPoints(n);
        // cells->Allocate(n); // 这种方式慢

        for (size_t i = 0; i < n; ++i) {
            const auto& p = block->m_points[i];
            points->SetPoint(i, p.x, p.y, p.z);
        }

        // 构建 Verts (每个点一个 Vertex)
        // 快速构建 cell array: 格式 [1, 0, 1, 1, 1, 2...] (1表示点数, 后面是索引)
        // vtkCellArray 较新版本有更快的接口，这里用兼容写法
        vtkNew<vtkIdList> ids;
        ids->SetNumberOfIds(1);
        for(vtkIdType i=0; i<n; ++i) {
            ids->SetId(0, i);
            cells->InsertNextCell(ids);
        }

        poly->SetPoints(points);
        poly->SetVerts(cells);

        // 颜色
        if (block->m_colors) {
            auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetNumberOfTuples(n);
            colors->SetName("Colors");

            // 获取内部指针
            unsigned char* ptr = colors->GetPointer(0);
            const auto& src_colors = *block->m_colors;

            // 使用 memcpy 直接拷贝整个数组，速度快几十倍
            if (sizeof(ct::RGB) == 3) {
                std::memcpy(ptr, src_colors.data(), n * 3);
            }
            else {
                // 如果结构体有 Padding (例如大小为4)，必须循环赋值
                for (size_t i = 0; i < n; ++i) {
                    const auto& c = src_colors[i];
                    size_t offset = i * 3;
                    ptr[offset]     = c.r;
                    ptr[offset + 1] = c.g;
                    ptr[offset + 2] = c.b;
                }
            }
            poly->GetPointData()->SetScalars(colors);
            if (poly->GetPointData()->GetScalars()) {
                poly->GetPointData()->SetActiveScalars("Colors");
            }
        }

        // 存入 Block 缓存
        vtkPolyData* rawPtr = poly.Get();
        if (rawPtr) {
            rawPtr->Register(nullptr);
        }

        block->m_vtk_polydata = std::shared_ptr<void>(rawPtr, [](void* p) {
            if (p) static_cast<vtkObject*>(p)->Delete();
        });
    }

    void OctreeRenderer::updateLODPolyData(OctreeNode* node)
    {
        // 逻辑同 updateBlockPolyData，只是源数据是 node->m_lod_points (PointXYZRGB)
        auto poly = vtkSmartPointer<vtkPolyData>::New();
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto cells = vtkSmartPointer<vtkCellArray>::New();
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");

        size_t n = node->m_lod_points.size();
        points->SetNumberOfPoints(n);
        colors->SetNumberOfTuples(n);

        vtkNew<vtkIdList> ids;
        ids->SetNumberOfIds(1);

        for (size_t i = 0; i < n; ++i) {
            const auto& p = node->m_lod_points[i];
            points->SetPoint(i, p.x, p.y, p.z);
            colors->SetTuple3(i, p.r, p.g, p.b);

            ids->SetId(0, i);
            cells->InsertNextCell(ids);
        }

        poly->SetPoints(points);
        poly->SetVerts(cells);
        poly->GetPointData()->SetScalars(colors);
        if (poly->GetPointData()->GetScalars()) {
            poly->GetPointData()->SetActiveScalars("Colors");
        }

        vtkPolyData* rawPtr = poly.Get();
        if (rawPtr) {
            rawPtr->Register(nullptr);
        }
        node->m_vtk_lod_polydata = std::shared_ptr<void>(rawPtr, [](void* p) {
            if (p) static_cast<vtkObject*>(p)->Delete();
        });
    }

    void invalidateRecursive(OctreeNode* node) {
        if (!node) return;

        // 标记 LOD 脏
        node->m_lod_dirty = true;
        // 释放显存缓存 (可选，或者等到下次渲染时覆盖)
        node->m_vtk_lod_polydata.reset();

        // 如果是叶子，标记 Block 脏
        if (node->isLeaf() && node->m_block) {
            node->m_block->m_is_dirty = true;
            node->m_block->m_vtk_polydata.reset();
        }

        // 递归
        for (int i = 0; i < 8; ++i) {
            if (node->m_children[i]) {
                invalidateRecursive(node->m_children[i]);
            }
        }
    }

    void OctreeRenderer::invalidateCache()
    {
        if (!m_cloud || !m_cloud->getOctreeRoot()) return;

        // 从根节点开始递归清理
        invalidateRecursive(m_cloud->getOctreeRoot());
    }

    std::vector<vtkActor*> OctreeRenderer::getActiveActors() const
    {
        std::vector<vtkActor*> actors;
        actors.reserve(m_used_actor_count);

        // 只有前 m_used_actor_count 个 Actor 是活跃且可见的
        for (int i = 0; i < m_used_actor_count; ++i) {
            actors.push_back(m_actor_pool[i].actor);
        }
        return actors;
    }

    CloudBlock::Ptr OctreeRenderer::getBlockFromActor(vtkActor* actor)
    {
        // 遍历 Actor 池查找
        // 这里的 m_actor_pool 大小通常在几百以内 (MAX_ACTORS)，线性查找非常快
        for (const auto& item : m_actor_pool) {
            if (item.actor == actor) {
                return item.block;
            }
        }
        return nullptr;
    }

} // namespace ct