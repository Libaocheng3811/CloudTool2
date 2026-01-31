//
// Created by LBC on 2026/1/28.
//

#ifndef CLOUDTOOL2_OCTREERENDERER_H
#define CLOUDTOOL2_OCTREERENDERER_H

#include "cloud.h"
#include "octree.h"

#include <vtkSmartPointer.h>
#include <vector>
#include <map>
#include <unordered_set>

// VTK 前向声明
class vtkRenderer;
class vtkActor;

namespace ct{
    /**
    * @brief 高性能八叉树 LOD 渲染器
    * @details 策略：
    * 1. SSE (Screen Space Error) 遍历：根据屏幕投影大小决定渲染精细 Block 还是粗糙 LOD。
    * 2. 动态合并 (Dynamic Merging)：每帧将所有可见点合并到一个巨大的 vtkPolyData 中，只使用 1 个 Actor。
    */
    class OctreeRenderer{
    public:
        OctreeRenderer(Cloud::Ptr cloud, vtkRenderer* renderer);
        ~OctreeRenderer();

        void setVisibility(bool visible);
        bool isVisible() const { return m_visible; }

        void update();
        void invalidateCache();

        std::vector<vtkActor*> getActiveActors() const;
        CloudBlock::Ptr getBlockFromActor(vtkActor* actor);
        ct::Cloud::Ptr getCloud() const {return m_cloud; }

    private:
        // 遍历上下文
        struct TraversalContext {
            const double* planes;
            Eigen::Vector3f camPos;
            float pixelsPerUnit;
            std::vector<OctreeNode*>* visibleNodes;
            int currentActorCount = 0; // 【新增】计数器
            int maxActors = 300;       // 【新增】最大允许渲染的块数
        };

        void traverse(OctreeNode* node, TraversalContext& ctx);

        // 为节点创建或获取缓存的 Actor
        vtkActor* getOrCreateActor(OctreeNode* node, bool is_lod);

        // 辅助计算
        bool isBoxInFrustum(const Box& box, const double* planes);
        float projectSize(const Box& box, const Eigen::Vector3f& camPos, float pixelsPerUnit);

    private:
        Cloud::Ptr m_cloud;
        vtkRenderer* m_vtk_renderer;
        bool m_visible = true;

        // --- 核心缓存 ---
        // Key: Node指针, Value: 对应的 VTK Actor
        // 我们不需要区分是 LOD 还是 Block，因为一个 Node 同一时间只显示一种状态
        // 但为了安全，我们可以让 Node 自己持有 Actor (侵入式) 或者这里用 Map (非侵入式)
        // 这里使用 Map 避免修改 OctreeNode 头文件太过频繁
        std::map<OctreeNode*, vtkSmartPointer<vtkActor>> m_actor_cache;

        // 记录上一帧显示的节点，用于差量更新
        std::unordered_set<OctreeNode*> m_current_visible_nodes;

        // --- 调度参数 ---
        float m_lod_threshold = 400.0f; // 屏幕投影像素阈值 (大于此值递归，小于此值画LOD)

        // 缓存上一帧的视点信息，减少不必要的更新
        Eigen::Vector3f m_last_cam_pos;
        double m_last_cam_dir[3];
        bool m_force_update = false;
    };

}// namespace ct

#endif //CLOUDTOOL2_OCTREERENDERER_H
