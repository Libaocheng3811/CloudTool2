//
// Created by LBC on 2026/1/28.
//

#ifndef CLOUDTOOL2_OCTREERENDERER_H
#define CLOUDTOOL2_OCTREERENDERER_H

#include "cloud.h"
#include "octree.h"

#include <vector>
#include <queue>
#include <map>

// VTK 前向声明
class vtkRenderer;
class vtkActor;
class vtkPolyDataMapper;
class vtkCamera;
class vtkPolyData;

namespace ct{
    /**
    * @brief 八叉树渲染调度器
    * @details 负责管理一个 Cloud 对象的渲染。实现视锥体剔除、LOD 切换和 Actor 池管理。
    */
    class OctreeRenderer{
    public:
        OctreeRenderer(Cloud::Ptr cloud, vtkRenderer* renderer);
        ~OctreeRenderer();

        void setVisibility(bool visible) { m_visible = visible; }

        bool isVisible() const { return m_visible; }
        // 每一帧渲染前调用 (响应相机移动)
        void update();

        // 强制刷新所有缓存 (例如修改颜色后)
        void invalidateCache();

        // 获取可见的 Actor 列表 (用于 Pick 等操作)
        std::vector<vtkActor*> getActiveActors() const;

        CloudBlock::Ptr getBlockFromActor(vtkActor* actor);

        ct::Cloud::Ptr getCloud() const {return m_cloud; }

    private:
        // --- 内部辅助结构 ---
        struct RenderItem {
            vtkActor* actor = nullptr;
            vtkPolyDataMapper* mapper = nullptr;
            CloudBlock::Ptr block; // 当前绑定的 Block (如果是 LOD 则为空)
            OctreeNode* lodNode = nullptr;   // 当前绑定的 LOD Node (如果是 Block 则为空)
            bool is_active = false; // 标记本帧是否被使用
        };

        // 递归遍历八叉树，收集可见节点
        void traverseOctree(OctreeNode* node, const double* planes,
                            const Eigen::Vector3f& camPos,
                            std::vector<CloudBlock::Ptr>& outVisibleBlocks,
                            std::vector<OctreeNode*>& outVisibleLODs);

        // 分配 Actor
        void assignActors(const std::vector<CloudBlock::Ptr>& blocks,
                          const std::vector<OctreeNode*>& lods);

        // 将数据转换为 vtkPolyData (懒加载核心)
        void updateBlockPolyData(CloudBlock* block);
        void updateLODPolyData(OctreeNode* node);

        // 检查包围盒是否在视锥体内
        bool isBoxInFrustum(const Box& box, const double* planes);

    private:
        Cloud::Ptr m_cloud;
        vtkRenderer* m_vtk_renderer; // 弱引用
        std::map<void*, int> m_block_to_actor_index; // 快速查找 Block 目前对应哪个 Actor 索引

        bool m_visible = true;

        // --- Actor 池 ---
        // 所有的 Actor 资源 (包括空闲的和正在使用的)
        std::vector<RenderItem> m_actor_pool;

        // 池大小限制 (例如 500 个块)
        // 如果可见块超过这个数，远处的块可能会被迫隐藏或合并 (暂未实现合并，直接丢弃最远的)
        static const int MAX_ACTORS = 2000;

        // 当前帧使用的 Actor 数量
        int m_used_actor_count = 0;

        // LOD 阈值 (距离/包围盒大小)
        float m_lod_threshold = 0.2f; // 屏幕投影大小比率
    };
}// namespace ct


#endif //CLOUDTOOL2_OCTREERENDERER_H
