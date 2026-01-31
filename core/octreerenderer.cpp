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
#include <vtkProperty.h>
#include <vtkIdTypeArray.h>
#include <omp.h>

namespace ct {

// 计算节点在屏幕上的投影大小（近似值）
    float screenSizeInPixels(const Box &box, const Eigen::Vector3f &camPos, vtkRenderer *renderer) {
        // 获取节点大小
        float size = std::max({box.width, box.height, box.depth});

        // 计算距离
        float dist = (box.translation - camPos).norm();
        if (dist < 1e-6f) return 1e6f;  // 非常近，返回大值

        // 简化的投影大小计算：size / distance * 视野因子
        // 这假设标准FOV，实际应该使用相机FOV
        float projected = size / dist;

        // 获取渲染器大小来转换为像素
        int *winSize = renderer->GetSize();
        float minDim = std::min(winSize[0], winSize[1]);

        // 放大因子，让投影值更直观
        return projected * minDim * 0.5f;
    }

    OctreeRenderer::OctreeRenderer(Cloud::Ptr cloud, vtkRenderer *renderer)
            : m_cloud(cloud), m_vtk_renderer(renderer) {}

    OctreeRenderer::~OctreeRenderer() {
        // 移除所有 Actor
        if (m_vtk_renderer) {
            for (auto& pair : m_actor_cache) {
                m_vtk_renderer->RemoveActor(pair.second);
            }
        }
        m_actor_cache.clear();
    }

    void OctreeRenderer::setVisibility(bool visible) {
        m_visible = visible;
        for (auto& pair : m_actor_cache) {
            // 如果整体不可见，隐藏所有；否则只显示当前活跃列表里的
            if (!visible) {
                pair.second->SetVisibility(0);
            } else if (m_current_visible_nodes.count(pair.first)) {
                pair.second->SetVisibility(1);
            }
        }
    }

    void OctreeRenderer::invalidateCache() {
        // 强制清理所有 Actor，通常用于颜色改变或点云重置
        for (auto& pair : m_actor_cache) {
            m_vtk_renderer->RemoveActor(pair.second);
        }
        m_actor_cache.clear();
        m_current_visible_nodes.clear();
        m_force_update = true;
    }

    std::vector<vtkActor*> OctreeRenderer::getActiveActors() const {
        std::vector<vtkActor*> actors;
        for (auto* node : m_current_visible_nodes) {
            if (m_actor_cache.count(node)) {
                actors.push_back(m_actor_cache.at(node));
            }
        }
        return actors;
    }

    CloudBlock::Ptr OctreeRenderer::getBlockFromActor(vtkActor* actor) {
        // 反查 (性能较低，仅用于拾取，平时不调用)
        for (auto& pair : m_actor_cache) {
            if (pair.second == actor) {
                return pair.first->m_block; // 注意：如果是 LOD 节点，这里 m_block 可能为空或是父节点
            }
        }
        return nullptr;
    }

    void OctreeRenderer::update()
    {
        // 0. 基础检查：如果没有点云、没有根节点或被设为不可见，直接返回
        if (!m_cloud || !m_cloud->getOctreeRoot() || !m_visible) return;

        vtkCamera* cam = m_vtk_renderer->GetActiveCamera();

        // ---------------------------------------------------------
        // 1. 相机状态检测 (Camera Change Detection)
        // ---------------------------------------------------------
        // 目的：如果画面静止，完全跳过计算，CPU占用降为0
        double* pos = cam->GetPosition();
        double* dir = cam->GetDirectionOfProjection();
        Eigen::Vector3f camPos(pos[0], pos[1], pos[2]);

        // 计算差异
        bool camChanged = (m_last_cam_pos - camPos).norm() > 0.01 ||
                          std::abs(m_last_cam_dir[0] - dir[0]) > 0.001 ||
                          std::abs(m_last_cam_dir[1] - dir[1]) > 0.001 ||
                          std::abs(m_last_cam_dir[2] - dir[2]) > 0.001;

        // 如果相机没动，且没有收到强制更新指令，直接退出
        if (!camChanged && !m_force_update) return;

        // 更新缓存
        m_last_cam_pos = camPos;
        m_last_cam_dir[0] = dir[0]; m_last_cam_dir[1] = dir[1]; m_last_cam_dir[2] = dir[2];
        m_force_update = false;

        // ---------------------------------------------------------
        // 2. 准备遍历参数 (Traversal Setup)
        // ---------------------------------------------------------
        double planes[24];
        cam->GetFrustumPlanes(m_vtk_renderer->GetTiledAspectRatio(), planes);

        int* winSize = m_vtk_renderer->GetSize();
        int height = winSize[1];
        double fov = cam->GetViewAngle();
        // 计算像素密度系数：用于将 3D 尺寸转换为屏幕像素大小
        float pixelsPerUnit = height / (2.0f * std::tan(fov * 0.5 * 0.0174532925));

        // 使用 Vector 进行遍历收集（保持遍历速度）
        std::vector<OctreeNode*> visibleNodeVector;
        // todo 将块的数量写死，不太好
        visibleNodeVector.reserve(2000); // 预估容量

        TraversalContext ctx;
        ctx.planes = planes;
        ctx.camPos = camPos;
        ctx.pixelsPerUnit = pixelsPerUnit;
        ctx.visibleNodes = &visibleNodeVector;

        // ---------------------------------------------------------
        // 3. 执行八叉树遍历 (Traverse)
        // ---------------------------------------------------------
        traverse(m_cloud->getOctreeRoot(), ctx);

        // ---------------------------------------------------------
        // 4. 差异化更新可见性 (Diff Update) - 核心优化
        // ---------------------------------------------------------

        // 构建下一帧的集合 (Set)，用于 O(1) 查找
        std::unordered_set<OctreeNode*> next_visible_set;
        next_visible_set.reserve(visibleNodeVector.size());

        // A. 处理【新增显示】的节点
        for (OctreeNode* node : visibleNodeVector) {
            next_visible_set.insert(node); // 填充 Set

            // 如果该节点上一帧不在显示列表中 -> 它是新来的，需要显示
            if (m_current_visible_nodes.find(node) == m_current_visible_nodes.end()) {
                // 获取或创建 Actor (如果 Actor 已缓存，这里非常快)
                // is_lod = true 如果不是叶子节点
                vtkActor* actor = getOrCreateActor(node, !node->isLeaf());
                if (actor) {
                    actor->SetVisibility(1);
                }
            }
            // 如果上一帧已经在显示列表中 -> 什么都不做 (Keep)，避免调用 VTK 造成开销
        }

        // B. 处理【需要隐藏】的节点
        // 遍历上一帧的所有节点
        for (OctreeNode* oldNode : m_current_visible_nodes) {
            // 如果旧节点不在新集合中 -> 它移出了视野或被 LOD 替换，需要隐藏
            if (next_visible_set.find(oldNode) == next_visible_set.end()) {
                // 查找 Actor 并隐藏 (不要销毁！保留在显存中供下次使用)
                auto it = m_actor_cache.find(oldNode);
                if (it != m_actor_cache.end()) {
                    it->second->SetVisibility(0);
                }
            }
        }

        // ---------------------------------------------------------
        // 5. 交换状态
        // ---------------------------------------------------------
        // 使用 move 避免拷贝，将当前帧状态更新为下一帧
        m_current_visible_nodes = std::move(next_visible_set);

        // 提示：此处不建议做 m_actor_cache 的清理（GC）。
        // 为了 CloudCompare 的那种流畅感，我们允许显存占用随浏览过的区域增加。
        // 只有在 Close 文件时才清理 m_actor_cache。
    }

    void OctreeRenderer::traverse(OctreeNode* node, TraversalContext& ctx)
    {
        if (!node) return;

        // 视锥体剔除
        if (!isBoxInFrustum(node->m_box, ctx.planes)) return;

        float size = projectSize(node->m_box, ctx.camPos, ctx.pixelsPerUnit);
        bool is_leaf = node->isLeaf();

        // 决策逻辑：
        // 如果投影像素 > 阈值 (例如 100px)，说明离得很近，需要看细节 -> 递归
        // 否则 -> 渲染当前节点的 LOD
        if (size > m_lod_threshold && node->hasChildren() && ctx.currentActorCount < ctx.maxActors) {
            for (int i=0; i<8; ++i) {
                if (node->m_children[i]) traverse(node->m_children[i], ctx);
            }
        } else {
            // 渲染当前节点
            // 如果是叶子，肯定渲染 Block
            // 如果是内部节点，渲染 LOD
            // 只要数据不为空就加入
            if (is_leaf) {
                if (node->m_block && !node->m_block->empty()) {
                    ctx.visibleNodes->push_back(node);
                    ctx.currentActorCount++;
                }
            } else {
                if (!node->m_lod_points.empty()) {
                    ctx.visibleNodes->push_back(node);
                    ctx.currentActorCount++;
                }
            }
        }
    }

    vtkActor* OctreeRenderer::getOrCreateActor(OctreeNode* node, bool is_lod)
    {
        // 1. 查找缓存
        auto it = m_actor_cache.find(node);
        if (it != m_actor_cache.end()) {
            return it->second;
        }

        // 2. 创建新 Actor
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        colors->SetNumberOfComponents(3);
        colors->SetName("Colors");

        // 根据类型填充数据
        if (is_lod) {
            const auto& src = node->m_lod_points;
            size_t n = src.size();
            points->SetNumberOfPoints(n);
            colors->SetNumberOfTuples(n);

            float* ptr_xyz = static_cast<float*>(points->GetVoidPointer(0));
            unsigned char* ptr_rgb = colors->GetPointer(0);

            for (size_t i = 0; i < n; ++i) {
                const auto& p = src[i];
                ptr_xyz[0]=p.x; ptr_xyz[1]=p.y; ptr_xyz[2]=p.z; ptr_xyz+=3;
                ptr_rgb[0]=p.r; ptr_rgb[1]=p.g; ptr_rgb[2]=p.b; ptr_rgb+=3;
            }
        } else {
            // Block
            auto block = node->m_block;
            size_t n = block->size();
            const auto& pts = block->m_points;
            const auto* cols = (block->m_colors) ? block->m_colors.get() : nullptr;

            points->SetNumberOfPoints(n);
            colors->SetNumberOfTuples(n);

            float* ptr_xyz = static_cast<float*>(points->GetVoidPointer(0));
            unsigned char* ptr_rgb = colors->GetPointer(0);

            for (size_t i = 0; i < n; ++i) {
                const auto& p = pts[i];
                ptr_xyz[0]=p.x; ptr_xyz[1]=p.y; ptr_xyz[2]=p.z; ptr_xyz+=3;
                if(cols) {
                    ptr_rgb[0]=(*cols)[i].r; ptr_rgb[1]=(*cols)[i].g; ptr_rgb[2]=(*cols)[i].b;
                } else {
                    ptr_rgb[0]=255; ptr_rgb[1]=255; ptr_rgb[2]=255;
                }
                ptr_rgb+=3;
            }
        }

        auto polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->GetPointData()->SetScalars(colors);

        // 生成 Vertex Cells (为了兼容性)
        vtkNew<vtkIdTypeArray> cells;
        cells->SetNumberOfValues(points->GetNumberOfPoints() * 2);
        vtkIdType* ids = cells->GetPointer(0);
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            ids[i*2] = 1;
            ids[i*2+1] = i;
        }
        vtkNew<vtkCellArray> cellArray;
        cellArray->SetCells(points->GetNumberOfPoints(), cells);
        polyData->SetVerts(cellArray);

        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);
        mapper->SetScalarModeToUsePointData();
        mapper->SetColorModeToDirectScalars();

        mapper->StaticOn();
        auto actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(m_cloud->pointSize());
        actor->GetProperty()->SetOpacity(m_cloud->opacity());
        actor->GetProperty()->BackfaceCullingOff();
        actor->SetVisibility(0); // 初始不可见

        m_vtk_renderer->AddActor(actor);
        m_actor_cache[node] = actor;

        return actor;
    }

    bool OctreeRenderer::isBoxInFrustum(const Box& box, const double* planes)
    {
        // AABB 视锥体剔除
        // 获取 Box 的 8 个角点
        double minX = box.translation.x() - box.width * 0.5;
        double maxX = box.translation.x() + box.width * 0.5;
        double minY = box.translation.y() - box.height * 0.5;
        double maxY = box.translation.y() + box.height * 0.5;
        double minZ = box.translation.z() - box.depth * 0.5;
        double maxZ = box.translation.z() + box.depth * 0.5;

        // 检查 6 个平面
        for (int i = 0; i < 6; ++i) {
            const double* p = planes + i * 4; // nx, ny, nz, d

            // p-vertex (方向一致的最远点)
            // 如果 p-vertex 在平面外（距离 < 0），则整个 Box 在平面外
            double px = (p[0] > 0) ? maxX : minX;
            double py = (p[1] > 0) ? maxY : minY;
            double pz = (p[2] > 0) ? maxZ : minZ;

            double dist = p[0]*px + p[1]*py + p[2]*pz + p[3];
            if (dist < 0) return false; // Cull
        }
        return true;
    }

    float OctreeRenderer::projectSize(const Box& box, const Eigen::Vector3f& camPos, float pixelsPerUnit)
    {
        // 计算包围盒中心到相机的距离
        float dist = (box.translation - camPos).norm();
        if (dist < 1e-6f) return 1e6f;

        // 近似投影大小：(包围盒最大边长 / 距离) * 像素密度
        float maxSize = std::max({box.width, box.height, box.depth});
        return (maxSize / dist) * pixelsPerUnit;
    }

} // namespace ct
