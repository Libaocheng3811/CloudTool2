# CLAUDE.md

此文件为 Claude Code (claude.ai/code) 提供在此代码库中工作的指导。

## 项目概述

CloudTool2 是一个基于 Qt5、VTK 和 PCL 构建的三维点云处理应用程序。提供点云可视化、滤波、地面/植被分割、变化检测和点云配准等功能。

## 构建系统

**环境要求**: CMake 3.28+, C++17, Qt5.15.2, VTK 9.1.0, PCL 1.12.1, OpenMP, Python 3.9 (EXACT), pybind11

```bash
# 配置（在项目根目录）
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# 构建
cmake --build build --config Release
```

**输出目录**:
- `build/bin/` - 可执行文件 (.exe) 和动态库 (.dll)
- `build/lib/` - 静态库 (.lib)

**编译器标志** (MSVC):
- `/MP` - 多处理器编译（根据 CPU 核心数自动设置）
- `/fp:precise` - 精确浮点运算
- `/bigobj` - 支持大对象文件
- `/wd4996` - 禁用 CRT 安全警告
- `/source-charset:utf-8` - 源文件 UTF-8 编码

**CMake 配置选项**:
```cmake
# Qt 路径（如需要手动指定）
set(CMAKE_PREFIX_PATH "D:/Qt5.15/5.15.2/msvc2019_64")

# OpenMP 支持
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

# Python 3.9（硬编码路径，需根据本地环境修改）
set(MY_NATIVE_PYTHON_DIR "F:/Program Files/Python39")
set(Python3_ROOT_DIR ${MY_NATIVE_PYTHON_DIR})
```

## 目录结构

```
CloudTool2/
├── 3rdparty/                    # 第三方库
│   ├── CSF/                    # 地面滤波算法（布料模拟）
│   ├── LAStools/               # LAS/LAZ 格式支持
│   │   ├── LASlib/             # LAS 读写库
│   │   └── LASzip/             # LAZ 压缩库
│   └── pybind11/               # Python C++ 绑定库（git submodule）
├── core/                        # 核心数据结构 (ct_core)
│   ├── cloud.h/cpp             # 点云数据结构（AOS 格式）
│   ├── cloudtype.h             # 点类型定义与压缩法线
│   ├── octree.h/cpp            # 八叉树空间索引 + LOD
│   ├── octreerenderer.h/cpp    # 高级渲染器（SSE 遍历）
│   ├── fileio.h/cpp            # 文件 I/O（流式加载）
│   ├── cloudview.h/cpp         # VTK 三维可视化控件
│   ├── console.h/cpp           # 日志输出
│   └── common.h                # 通用工具函数
├── python/                      # 嵌入式 Python 模块 (ct_python)
│   ├── python_manager.h/cpp    # Python 解释器生命周期管理（单例）
│   ├── python_worker.h/cpp     # QThread 脚本执行引擎（GIL + 异步取消）
│   ├── python_bridge.h/cpp     # 信号桥接 + 线程安全云注册表
│   ├── python_bindings.cpp     # pybind11 嵌入模块 `ct`（PyCloud + 工厂函数）
│   └── CMakeLists.txt          # OBJECT 库构建配置
├── modules/                     # 算法模块 (ct_modules)
│   ├── filters.h/cpp           # 滤波算法
│   ├── features.h/cpp          # 特征提取
│   ├── registration.h/cpp      # 点云配准
│   ├── keypoints.h/cpp         # 关键点检测
│   ├── csffilter.h/cpp         # CSF 地面分割
│   ├── vegfilter.h/cpp         # 植被分割
│   ├── distancecalculator.h/cpp # 距离计算（变化检测）
│   └── utils.h                 # 算法工具函数
├── widgets/                     # UI 组件
│   ├── customdialog.h          # 对话框基类
│   ├── customdock.h            # Dock 基类
│   ├── customtree.h            # 树控件基类
│   ├── cloudtree.h/cpp         # 点云树管理
│   └── common_ui/              # 通用对话框 (ct_common_ui)
│       ├── processingdialog.h         # 进度条
│       ├── globalshiftdialog.h        # 全局偏移设置
│       ├── fieldmappingdialog.h       # 字段映射
│       ├── txtimportdialog.h          # TXT 导入
│       └── txtexportdialog.h          # TXT 导出
├── cloudtool/                   # 应用层
│   ├── main.cpp                # 程序入口
│   ├── mainwindow.h/cpp        # 主窗口
│   ├── edit/                   # 编辑工具
│   │   ├── color.h/cpp         # 颜色设置
│   │   └── boundingbox.h/cpp   # 包围盒绘制
│   ├── tool/                   # 处理工具
│   │   ├── filters.h/cpp       # 滤波工具
│   │   ├── registration.h/cpp  # 配准工具
│   │   ├── keypoints.h/cpp     # 关键点工具
│   │   ├── cutting.h/cpp       # 裁剪工具
│   │   ├── pickpoints.h/cpp    # 点拾取工具
│   │   ├── descriptor.h/cpp    # 描述符工具
│   │   └── rangeimage.h/cpp    # 深度图工具
│   ├── plugins/                # 插件系统
│   │   ├── csfplugin.h/cpp     # CSF 地面分割插件
│   │   ├── vegplugin.h/cpp     # 植被分割插件
│   │   └── changedetectplugin.h/cpp # 变化检测插件
│   └── resources/              # 资源文件
│       ├── res.qrc             # Qt 资源文件
│       └── logo/               # 图标与 Logo
├── camera/                     # 相机 SDK（可选）
│   ├── AzureKinect/            # Azure Kinect SDK
│   └── Photoneo/               # Photoneo SDK
├── data/                       # 测试数据
├── CMakeLists.txt              # 主构建配置
├── CLAUDE.md                   # 本文件
└── README.md                   # 项目说明
```

## 架构设计

项目采用 **三层架构**:

```
┌─────────────────────────────────────────────┐
│ Level 3: 应用层 (cloudtool/)                 │
│ MainWindow, Tools, Plugins                  │
│ 业务逻辑与 UI 结合                           │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────┴──────────────────────────┐
│ Level 2: UI 组件层 (widgets/)               │
│ CustomDialog, CustomDock, CloudTree         │
│ 通用对话框                                   │
└──────────────────┬──────────────────────────┘
                   │
┌──────────────────┴──────────────────────────┐
│ Level 1: 核心与算法层                        │
│ ct_core: Cloud, Octree, FileIO, CloudView   │
│ ct_modules: Filters, Features, Registration │
└─────────────────────────────────────────────┘
```

### 依赖关系

```
cloudtool
    ↓ depends on
ct_widget ←→ ct_common_ui
    ↓ depends on
ct_python (OBJECT) ←→ ct_modules
    ↓               ↓ depends on
    ↓ depends on    ct_core
ct_core              ↓ depends on
    ↓ depends on    Qt5, PCL
Qt5, VTK, PCL, OpenMP, pybind11, Python3
```

## 核心组件详解

### Cloud (core/cloud.h)

主要点云数据结构，采用 **AOS (Array of Structures)** 格式以节省内存。

**关键特性**:
- 从 SOA 重构为 AOS（提交 192e688），可处理更大的点云文件
- 基于八叉树的空间索引，`CloudBlock` 叶子节点最多存储 6 万个点
- 支持多种点类型：XYZ、XYZRGB、XYZNormal、XYZRGBNormal
- 标量字段管理 (`m_scalar_cache`)，支持自定义属性
- PCL 互操作性：`toPCL_XYZRGB()`、`fromPCL_XYZRGB()` 等方法

**核心数据成员**:
```cpp
class Cloud {
    OctreeNode::Ptr m_octree_root;          // 八叉树根节点
    std::vector<CloudBlock::Ptr> m_all_blocks;  // 扁平化 Block 列表
    CloudConfig m_config;                   // 配置参数
    QMap<QString, std::vector<float>> m_scalar_cache;  // 标量场缓存
    Eigen::Vector3d m_global_shift;         // 全局坐标偏移
};
```

**自适应配置** (`calculateAdaptiveConfig`):
- 点数 < 1000 万：直通模式（禁用八叉树）
- 点数 >= 1000 万：八叉树模式，自动计算 Block 大小和 LOD 参数

### CloudBlock (core/cloud.h)

八叉树叶子节点负载，存储实际点云数据。

**数据结构**:
```cpp
class CloudBlock {
    std::vector<pcl::PointXYZ> m_points;     // 3D 坐标（必需）
    std::unique_ptr<std::vector<RGB>> m_colors;  // RGB 颜色（可选）
    std::unique_ptr<std::vector<CompressedNormal>> m_normals;  // 压缩法线
    QMap<QString, std::vector<float>> m_scalar_fields;  // 标量场

    Box m_box;                               // 包围盒
    std::shared_ptr<void> m_vtk_polydata;    // VTK 缓存
    bool m_is_dirty = true;                  // 脏标记
};
```

### Octree (core/octree.h)

空间分割与 LOD 生成。

**OctreeNode**:
```cpp
class OctreeNode {
    OctreeNode* m_parent;                    // 父节点
    OctreeNode* m_children[8];               // 8 个子节点
    CloudBlock::Ptr m_block;                 // 叶子节点数据
    std::vector<pcl::PointXYZRGB> m_lod_points;  // LOD 数据
    Box m_box;                               // 包围盒
    int m_depth;                             // 深度
    size_t m_total_points_in_node;           // 点数统计
};
```

**LOD 生成策略** (蓄水池采样 Reservoir Sampling):
```cpp
// 流式插入时实时更新 LOD
if (node->m_lod_points.size() < capacity) {
    node->m_lod_points.push_back(point);
} else {
    // 随机替换概率 = k/n
    if (rand() % current_n < capacity) {
        node->m_lod_points[rand() % capacity] = point;
    }
}
```

### OctreeRenderer (core/octreerenderer.h)

高性能渲染器，采用 SSE 遍历策略。

**渲染特性**:
- **SSE (Screen Space Error)** 遍历：根据屏幕投影大小决定渲染精细度
- **视锥剔除** (Frustum Culling)：只渲染可见节点
- **动态阈值**：交互时降低质量，静止时提高质量
- **Actor 对象池**：复用 VTK Actor（最多 500 个），减少 GPU 资源创建开销

**渲染流程**:
```cpp
void OctreeRenderer::update() {
    // 1. 检查相机是否移动
    if (!camChanged && !m_force_update) return;

    // 2. 优先队列遍历（SSE 策略）
    std::priority_queue<PriorityNode> pq;
    pq.push({root, projectSize(root->m_box)});

    while (!pq.empty()) {
        auto node = pq.top().node;
        float screenSize = projectSize(node->m_box);

        // 判断是否需要细分
        if (screenSize > m_base_threshold && node->hasChildren()) {
            // 继续细分子节点
        } else {
            // 渲染此节点（LOD 或 Block）
            vtkActor* actor = getOrCreateActor(node, isLOD);
        }
    }
}
```

### FileIO (core/fileio.h)

流式文件加载与保存。

**支持格式**:
- LAS/LAZ（通过 LASlib）
- PLY、PCD
- TXT（支持字段映射）
- OBJ（只读）

**流式加载流程**:
```cpp
bool FileIO::loadLAS(const QString& filename, Cloud::Ptr& cloud) {
    // 1. 读取 Header 获取包围盒
    LASreader* lasreader = lasreadopener.open(filename);

    // 2. 初始化八叉树
    cloud->initOctree(globalBox);

    // 3. 流式读取（每批 50 万点）
    CloudBatch batch;
    batch.reserve(BATCH_SIZE);

    while (lasreader->read_point()) {
        batch.points.push_back(pt);
        if (batch.points.size() >= BATCH_SIZE) {
            batch.flushTo(cloud);
            emit progress(percent);
        }
    }

    // 4. 生成 LOD
    cloud->generateLOD();
}
```

**全局坐标偏移**:
```cpp
// 问题：UTM 坐标 (x=500000) 导致 GPU 精度丢失
// 解决：渲染前减去质心，保存时加回
Eigen::Vector3d shift = calculateCentroid(cloud);
cloud->setGlobalShift(shift);
```

### CloudView (core/cloudview.h)

基于 VTK 的三维可视化控件。

**功能**:
- 交互式点拾取（单点和多边形区域选择）
- 相机控制和视口管理
- 支持添加形状、箭头、标签、对应关系
- 多点云渲染管理

### CloudType (core/cloudtype.h)

点类型定义与数据结构。

**类型定义**:
```cpp
using RGB = pcl::RGB;
struct PointXYZ { float x, y, z; };

// 压缩法线（节省内存）
struct CompressedNormal {
    int nx : 11;  // [-1, 1] -> [0, 2047]
    int ny : 11;
    int nz : 10;
};
```

## 算法模块详解

### Filters (modules/filters.h)

支持的滤波器：

| 滤波器 | 说明 |
|--------|------|
| PassThrough | 直通滤波（阈值裁剪） |
| VoxelGrid | 体素降采样 |
| StatisticalOutlierRemoval | 统计离群点移除 |
| RadiusOutlierRemoval | 半径离群点移除 |
| ConditionalRemoval | 条件滤波 |
| GridMinimum | 2D 网格最小值投影 |
| LocalMaximum | 局部最大值移除 |

### Features (modules/features.h)

支持的特征描述符：

| 描述符 | 全称 |
|--------|------|
| PFH | Point Feature Histogram |
| FPFH | Fast Point Feature Histograms |
| VFH | Viewpoint Feature Histogram |
| SHOT | Signature of Histograms of OrienTations |
| ESF | Ensemble of Shape Functions |
| GASD | Globally Aligned Spatial Distribution |

### Registration (modules/registration.h)

配准算法：

| 算法 | 说明 |
|------|------|
| ICP | 迭代最近点算法 |
| ICPWithNormals | 带法线的 ICP |
| ICPNonLinear | 非线性 ICP |
| IA-RANSAC | SampleConsensusInitialAlignment |
| SCPR | SampleConsensusPrerejective |
| FPCS | FPCSInitialAlignment |
| KFPCS | KFPCSInitialAlignment |
| NDT | Normal Distributions Transform |

### Keypoints (modules/keypoints.h)

关键点检测算法：

| 算法 | 说明 |
|------|------|
| ISS | Intrinsic Shape Signatures |
| Harris3D | Harris 3D 关键点 |
| SIFT3D | SIFT 3D |
| NARF | Normal Aligned Radial Features |

### CSFFilter (modules/csffilter.h)

布料模拟地面滤波器。

**参数**:
- `bSloopSmooth` - 坡度平滑
- `time_step` - 时间步长
- `class_threshold` - 分类阈值
- `cloth_resolution` - 布料分辨率
- `rigidness` - 布料硬度 (1-3)
- `iterations` - 迭代次数

### VegFilter (modules/vegfilter.h)

植被分割滤波器。

**植被指数类型**:
```cpp
enum class VegIndexType {
    ExG_ExR,  // Excess Green - Excess Red
    ExG,      // Excess Green: 2g - r - b
    NGRDI,    // (g - r) / (g + r)
    CIVE      // 0.441r - 0.811g + 0.385b + 18.787
};
```

**阈值策略**:
- 用户手动设置
- Otsu 自动阈值（大津法）

### DistanceCalculator (modules/distancecalculator.h)

点云距离计算（变化检测）。

**距离计算方法**:
```cpp
enum Method {
    C2C_NEAREST = 0,      // 最近邻距离
    C2C_KNN_MEAN = 1,     // K 近邻平均距离
    C2C_RADIUS_MEAN = 2,  // 半径内平均距离
    C2M_SIGNED = 3,       // 点到网格有符号距离（预留）
    M3C2 = 4              // M3C2 算法（预留）
};
```

## 插件系统

### 插件模式

所有插件遵循统一的模板：

```cpp
class Plugin : public ct::CustomDialog {
    QThread m_thread;           // 工作线程
    Worker* m_worker;           // 算法实例
    ct::Cloud::Ptr m_cloud;     // 输入点云

    void init() override;       // 设置 UI 连接
    void onApply();             // 开始处理
    void onDone(Result);        // 处理完成
};
```

**工作线程模式**: 耗时操作在 `QThread` 中运行：
- `progress(int percent)` 信号用于进度跟踪
- 原子标志 `m_is_canceled` 用于协作式取消
- 模态对话框 `ProcessingDialog` 提供用户反馈

### CSFPlugin (cloudtool/plugins/csfplugin.h)

地面点分割插件。

### VegPlugin (cloudtool/plugins/vegplugin.h)

植被分割插件，支持：
- 4 种植被指数
- Otsu 自动阈值
- 手动阈值输入

### ChangeDetectPlugin (cloudtool/plugins/changedetectplugin.h)

变化检测插件：
- 参考点云 vs 比较点云
- 多种距离计算方法
- 阈值过滤
- Jet 色带颜色映射

## 嵌入式 Python 集成 (python/)

CloudTool2 通过 pybind11 embed 模式嵌入 Python 3.9 解释器，允许用户通过 Python 脚本访问和操作点云数据。

### 解释器生命周期

```cpp
// cloudtool/main.cpp
int main() {
    QApplication app;
    ct::PythonManager::instance().initialize();  // Py_Initialize + 注册模块
    MainWindow w;
    w.show();
    int ret = app.exec();
    ct::PythonManager::instance().finalize();     // 清理（不调用 Py_Finalize）
    return ret;
}
```

**关键约束**: 不调用 `Py_Finalize()`，避免 pybind11 全局析构顺序问题。

### 架构三要素

| 组件 | 职责 |
|------|------|
| `PythonManager` | 单例，管理解释器初始化/销毁、stdio 重定向、DLL 搜索路径 |
| `PythonWorker` | QThread，GIL 管理的脚本执行（`PyGILState_Ensure/Release`），支持异步取消 |
| `PythonBridge` | 信号桥接 + 线程安全云注册表。Python 侧只发信号，不直接操作 UI |

### 线程安全规则

1. **GIL**: Python 代码只在 `PythonWorker::run()` 中持有 GIL 执行；主线程释放 GIL
2. **Capsule 生命周期**: 零拷贝 NumPy view 通过 `py::capsule` 持有 `Cloud::Ptr`，防止点云被提前销毁
3. **信号-only UI**: Python 侧所有 UI 操作通过 `PythonBridge` 发射信号，由 Qt 主线程处理

### Python API (`import ct`)

```python
import ct

# 日志输出到 GUI Console
ct.printI("info message")
ct.printW("warning")
ct.printE("error")

# 获取点云（线程安全，自动 hold + mark in-use）
cloud = ct.get_cloud("my_cloud")  # 返回 ct.Cloud 或 None

# 按 Block 零拷贝访问（高性能，适用于大点云）
for i in range(cloud.num_blocks()):
    xyz = cloud.block_to_numpy(i)       # shape (M, 3), float32 零拷贝
    colors = cloud.block_get_colors(i)  # shape (M, 3), uint8 零拷贝

    # 修改后写回
    cloud.block_set_colors(i, new_colors)  # 拷贝写入
    cloud.block_set_numpy(i, new_xyz)      # 拷贝写入
    cloud.block_mark_dirty(i)              # 重算包围盒
    cloud.refresh()                        # 触发 VTK 重绘

# 全量拷贝（合并所有 Block）
xyz_all = cloud.to_numpy()        # shape (N, 3), float32
colors_all = cloud.get_colors()   # shape (N, 3), uint8

# 属性查询
cloud.size()          # 总点数
cloud.has_colors()    # 是否有颜色
cloud.has_normals()   # 是否有法线
```

### In-use 保护机制

```cpp
// Python 获取云时自动标记 in-use，防止 CloudTree 删除正在使用的云
bridge->holdCloud(cloud);           // 持有 shared_ptr 引用
bridge->markCloudInUse(cloud->id()); // 标记 UI 侧禁止删除

// Python 脚本执行完成后自动释放
bridge->releaseAllHeld();           // 释放引用
bridge->releaseAllInUse();          // 取消删除保护
```

### 构建配置

- **ct_python** 为 OBJECT 库（非 STATIC/SHARED），编译产物直接链接到 cloudtool 可执行文件
- pybind11 通过 `3rdparty/pybind11` git submodule 引入
- Python 路径硬编码为 `MY_NATIVE_PYTHON_DIR`，需根据本地环境修改
- `#undef slots` 必须在包含 pybind11 头文件之前，解决 Qt `slots` 宏与 Python `object.h` 的冲突

## UI 组件

### CustomDialog (widgets/customdialog.h)

对话框基类。

**特性**:
- 依赖注入模式（通过构造函数注入 CloudView、CloudTree、Console）
- 自动位置管理（工具浮窗跟随 CloudView）
- 单例管理（`registed_dialogs` 全局注册表）

**使用模式**:
```cpp
// 创建工具浮窗（无边框、跟随视图）
createDialog<Filters>(parent, "Filters", cloudview, cloudtree, console,
                      true, false);

// 创建模态对话框（阻塞、居中）
createModalDialog<GlobalShiftDialog>(parent, "Global Shift", ...);
```

### CloudTree (widgets/cloudtree.h)

点云树管理控件。

**核心功能**:
- 点云添加/删除/保存
- 节点克隆/合并
- 右键菜单管理
- 进度条绑定

**数据结构**:
```cpp
class CloudTree {
    QMap<QTreeWidgetItem*, Cloud::Ptr> m_cloud_map;  // Item -> Cloud 映射
    QThread m_thread;                                 // 文件 I/O 线程
    FileIO* m_fileio;                                 // 文件 I/O 实例
    ProcessingDialog* m_processing_dialog;            // 进度对话框
};
```

### 通用对话框

| 对话框 | 文件 | 功能 |
|--------|------|------|
| ProcessingDialog | common_ui/processingdialog.h | 进度条 + 取消按钮 |
| GlobalShiftDialog | common_ui/globalshiftdialog.h | 全局偏移设置 + 记忆 |
| FieldMappingDialog | common_ui/fieldmappingdialog.h | TXT 字段映射 |
| TxtImportDialog | common_ui/txtimportdialog.h | TXT 导入配置 |
| TxtExportDialog | common_ui/txtexportdialog.h | TXT 导出配置 |

## 工具

### Filters Tool (cloudtool/tool/filters.h)

滤波工具：
- 滤波器类型选择（Tab 页）
- 参数配置（SpinBox/DoubleSpinBox）
- 预览/应用/添加按钮

### Registration Tool (cloudtool/tool/registration.h)

配准工具：
- 源点云和目标点云选择
- 配准参数配置
- 配准结果显示（变换矩阵、Fitness Score）
- 对应关系可视化

### Keypoints Tool (cloudtool/tool/keypoints.h)

关键点检测工具。

### Cutting Tool (cloudtool/tool/cutting.h)

裁剪工具：
- 包围盒裁剪
- 多边形裁剪

### PickPoints Tool (cloudtool/tool/pickpoints.h)

点拾取工具：
- 单点拾取
- 多边形区域选择
- 保存到文件

### RangeImage Tool (cloudtool/tool/rangeimage.h)

深度图工具：
- 生成 RangeImage
- 边界提取
- 法线估计

### Descriptor Tool (cloudtool/tool/descriptor.h)

描述符工具。

### Edit Tools (cloudtool/edit/)

- **color.h/cpp** - 颜色设置
- **boundingbox.h/cpp** - 包围盒绘制

## 相机 SDK 支持

### Azure Kinect (camera/AzureKinect/)

Microsoft Azure Kinect DK 深度相机支持。

**头文件**: `include/k4a/k4a.h`
**库目录**: `lib/win32` / `lib/linux`

### Photoneo (camera/Photoneo/)

Photoneo 3D 扫描仪支持。

**头文件**: `include/`
**库目录**: `lib/win32` / `lib/linux`

## 大点云处理策略

### 1. 八叉树分区
- 空间递归划分为 8 个子空间
- 叶子节点最多存储 6 万个点（可配置）
- 非连续内存分配

### 2. 流式 I/O
- 批量加载（每批 50 万点）
- 实时进度反馈
- 内存峰值控制

### 3. 全局坐标偏移
```cpp
// 问题：UTM 坐标 (x=500000) 导致 GPU 精度丢失
// 解决：渲染前减去质心
pt.x -= shift.x();
pt.y -= shift.y();
pt.z -= shift.z();

// 保存时加回
pt.x += shift.x();
```

### 4. 动态 LOD
- 交互态：降低阈值，渲染粗糙 LOD
- 静止态：提高阈值，渲染精细 Block
- SSE 策略：根据屏幕投影大小动态选择

### 5. 视锥剔除
```cpp
bool isBoxInFrustum(const Box& box, const double* planes) {
    for (int i = 0; i < 6; ++i) {  // 6 个裁剪面
        // 检查 8 个顶点是否都在平面外侧
        // ...
    }
}
```

## 设计模式

| 模式 | 应用位置 |
|------|----------|
| 工厂模式 | 对话框创建 (`createDialog<T>`) |
| 观察者模式 | 信号/槽机制 |
| 策略模式 | 配准算法选择 |
| 单例模式 | 对话框管理 (`registed_dialogs`) |
| 模板方法 | 插件架构 (`init()` + `onApply()`) |

## 渲染管线

```
1. 数据存储在 CloudBlock 结构中（AOS 格式）
2. OctreeRenderer 管理 VTK actor 对象池
3. 视锥剔除确定可见节点
4. SSE 遍历选择合适的 LOD 级别
5. Actor 池复用 GPU 资源（最多 500 个）
```

## Git 提交规范

遵循约定式提交格式：

```
<type>(<scope>): <description>
```

**类型 (type)**:
- `feat` - 新功能
- `fix` - 缺陷修复
- `docs` - 文档更新
- `style` - 代码格式
- `refactor` - 代码重构
- `perf` - 性能优化
- `test` - 测试相关
- `chore` - 构建/工具

**示例**:
```
feat(ui): 添加导出按钮
fix(core): 修复空指针崩溃
refactor(render): 优化八叉树渲染性能
chore(build): 更新 cmake 版本
```

## 文件格式支持

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | √ | √ | ASPRS LAS 格式 |
| LAZ | .laz | √ | √ | 压缩 LAS 格式 |
| PLY | .ply | √ | √ | Stanford PLY 格式 |
| PCD | .pcd | √ | √ | PCL 点云格式 |
| TXT | .txt | √ | √ | 文本格式（支持字段映射） |
| OBJ | .obj | √ | - | Wavefront OBJ 格式 |

## 预处理器定义

| 宏 | 值 | 说明 |
|----|----|----|
| `ROOT_PATH` | 项目根目录 | 项目源码路径 |
| `DATA_PATH` | data/ | 数据文件路径 |
| `PYTHON_HOME` | Python 安装目录 | 嵌入式 Python 解释器路径 |
| `CT_LIBRARY` | - | 标记库导出（用于 Windows DLL） |

## 重要实现说明

### 全局坐标偏移
**问题**: 大坐标（如 UTM）导致 GPU 精度问题（抖动/卡顿）
**解决方案**: 渲染前减去质心，保存时加回
**配置**: `GlobalShiftDialog` 允许用户配置/记忆偏移值

### PCL 集成
- 内部使用 PCL 点云进行算法处理
- 完整的 PCL 转换在大点云下可能导致内存问题，需选择性使用
- 自定义 `Cloud` 类封装 PCL，添加八叉树索引

### AOS vs SOA
- **旧版本 (SOA)**: `std::vector<float> x, y, z;` - 多次内存跳转
- **新版本 (AOS)**: `std::vector<PointXYZ> points;` - 连续内存访问

## CMake 库配置

### ct_core (core/CMakeLists.txt)
```cmake
add_library(ct_core SHARED ${LibSrcs} ${LibHdrs})
target_link_libraries(ct_core
    PUBLIC Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES}
    PRIVATE LASlib OpenMP::OpenMP_CXX)
target_compile_definitions(ct_core PRIVATE CT_LIBRARY)
```

### ct_modules (modules/CMakeLists.txt)
```cmake
add_library(ct_modules SHARED ${LibSrcs} ${LibHdrs})
target_link_libraries(ct_modules
    PRIVATE ct_core CSF_Lib Qt5::Widgets ${PCL_LIBRARIES})
```

### ct_widget & ct_common_ui (widgets/CMakeLists.txt)
```cmake
add_library(ct_common_ui STATIC ${UI_SOURCES} ${UI_HEADERS} ${UI_FORMS})
add_library(ct_widget STATIC ${Widget_Srcs} ${Widget_Hdrs})
target_link_libraries(ct_widget PUBLIC ct_common_ui ct_core)
```

### cloudtool (cloudtool/CMakeLists.txt)
```cmake
add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs})
target_link_libraries(cloudtool
    PRIVATE ct_core ct_widget ct_modules ct_python
            Qt5::Widgets ${VTK_LIBRARIES} pybind11::embed Python3::Python)
```

## 第三方库配置

### LAStools
```cmake
# 调用式配置
set(LASZIP_BUILD_STATIC ON CACHE BOOL "Build static laszip" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "Build static laslib" FORCE)
add_subdirectory(3rdparty/LAStools)

# 链接
target_link_libraries(ct_core PRIVATE LASlib)
```

### CSF
```cmake
# 托管式配置（自定义 CMakeLists）
# 3rdparty/CSF/CMakeLists.txt:
file(GLOB CSF_Srcs "src/*.cpp")
file(GLOB CSF_Hdrs "src/*.h")
add_library(CSF_Lib STATIC ${CSF_Srcs} ${CSF_Hdrs})
target_link_libraries(CSF_Lib PRIVATE OpenMP::OpenMP_CXX)

# 主程序链接
target_link_libraries(ct_modules PRIVATE CSF_Lib)
```

### pybind11
```cmake
# git submodule 方式引入
add_subdirectory(3rdparty/pybind11)
# ct_python 通过 pybind11::embed 链接
```

## 常见问题

### Q: 编译时找不到 Qt5/VTK/PCL？
A: 请确保已正确设置 `CMAKE_PREFIX_PATH` 或环境变量。

### Q: 大点云文件打开缓慢？
A: 已启用流式加载和 LOD 渲染，可尝试调整 `CloudConfig` 中的参数。

### Q: UTM 坐标渲染时抖动？
A: 使用全局坐标偏移功能，在导入时设置合适的偏移值。

### Q: 内存占用过高？
A: 检查是否启用了八叉树模式，调整 `maxPointsPerBlock` 参数。

### Q: Python 初始化失败？
A: 检查 `MY_NATIVE_PYTHON_DIR` 路径是否正确指向 Python 3.9 安装目录。pybind11 要求版本精确匹配。

### Q: `#undef slots` 编译错误？
A: pybind11 头文件必须在 `#undef slots` 之后包含（见 `python_bindings.cpp`），因为 Qt 的 `slots` 宏与 Python `object.h` 冲突。

## 开发指南

### 添加新的滤波器
1. 在 `modules/filters.h` 中添加滤波器枚举
2. 在 `Filters::apply()` 中实现滤波逻辑
3. 在 `cloudtool/tool/filters.h` 中添加 UI 配置

### 添加新的插件
1. 继承 `ct::CustomDialog` 类
2. 实现 `init()`, `onApply()`, `onDone()` 方法
3. 在 `MainWindow` 中注册插件

### 添加新的文件格式
1. 在 `core/fileio.h` 中添加加载/保存函数
2. 在 `CloudTree` 中添加格式过滤器
3. 测试流式加载和坐标偏移

### 扩展 Python API
1. 在 `python/python_bindings.cpp` 的 `PYBIND11_EMBEDDED_MODULE(ct, m)` 中添加新函数或类
2. 零拷贝访问使用 `py::capsule` 绑定 `Cloud::Ptr` 生命周期
3. UI 操作只通过 `PythonBridge` 发射信号，禁止 Python 线程直接操作 UI
4. 需要新的 UI 信号时，在 `PythonBridge` 中添加信号定义 + `MainWindow` 中 connect

---

**维护者**: 请在修改核心组件时更新此文档
**最后更新**: 2026-03-30
