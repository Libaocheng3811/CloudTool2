# CloudTool2

> 一个基于 Qt5、VTK 和 PCL 构建的专业三维点云处理应用程序

## 简介

CloudTool2 是一个功能强大的点云处理软件，提供点云可视化、滤波、地面/植被分割、变化检测和点云配准等功能。项目采用三层架构设计，支持大规模点云数据的流式加载和高效渲染。

## 功能特性

### 核心功能
- **点云可视化** - 基于 VTK 的三维交互式可视化
- **数据导入/导出** - 支持 LAS/LAZ、PLY、PCD、TXT、OBJ 等多种格式
- **滤波处理** - 直通滤波、体素降采样、统计离群点移除等
- **点云配准** - ICP、IA-RANSAC、NDT 等多种配准算法
- **特征提取** - PFH、FPFH、VFH、SHOT 等特征描述符
- **地面分割** - CSF 布料模拟滤波
- **植被分割** - 多种植被指数 (ExG、NGRDI、CIVE)
- **变化检测** - 点云距离计算与分析

### 大点云处理
- **八叉树空间索引** - 高效的空间分割与查询
- **流式加载** - 批量加载避免内存峰值
- **LOD 渲染** - 动态细节层次，提升交互性能
- **全局坐标偏移** - 解决 UTM 大坐标的 GPU 精度问题

## 环境要求

| 依赖项 | 最低版本 | 推荐版本 |
|--------|----------|----------|
| CMake | 3.28 | 3.28+ |
| C++ 标准 | C++17 | C++17 |
| Qt5 | 5.15.2 | 5.15.2 |
| VTK | 9.1.0 | 9.1.0 |
| PCL | 1.12.1 | 1.12.1 |
| OpenMP | - | 最新版 |
| 编译器 | MSVC 2019+ | MSVC 2022 |

## 编译构建

### Windows (MSVC)

```bash
# 1. 配置项目
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release

# 2. 编译
cmake --build build --config Release

# 3. 输出目录
# 可执行文件: build/bin/cloudtool.exe
# 静态库: build/lib/
```

### CMake 配置选项

```bash
# 设置 Qt 路径（如需要）
set(CMAKE_PREFIX_PATH "D:/Qt5.15/5.15.2/msvc2019_64")

# OpenMP 支持
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)
```

## 项目架构

```
CloudTool2/
├── 3rdparty/              # 第三方库
│   ├── CSF/              # 地面滤波算法
│   └── LAStools/         # LAS/LAZ 格式支持
├── core/                  # 核心数据结构 (ct_core)
│   ├── cloud.h/cpp       # 点云数据结构
│   ├── octree.h/cpp      # 八叉树空间索引
│   ├── octreerenderer.h/cpp # 高级渲染器
│   ├── fileio.h/cpp      # 文件 I/O
│   ├── cloudview.h/cpp   # 3D 可视化控件
│   └── console.h/cpp     # 日志输出
├── modules/              # 算法模块 (ct_modules)
│   ├── filters.h/cpp     # 滤波算法
│   ├── features.h/cpp    # 特征提取
│   ├── registration.h/cpp # 点云配准
│   ├── csffilter.h/cpp   # CSF 地面分割
│   ├── vegfilter.h/cpp   # 植被分割
│   └── distancecalculator.h/cpp # 距离计算
├── widgets/              # UI 组件 (ct_widget, ct_common_ui)
│   ├── customdialog.h    # 对话框基类
│   ├── cloudtree.h/cpp   # 点云树管理
│   └── common_ui/        # 通用对话框
├── cloudtool/            # 应用层
│   ├── mainwindow.h/cpp  # 主窗口
│   ├── edit/             # 编辑工具
│   ├── tool/             # 处理工具
│   ├── plugins/          # 插件系统
│   └── resources/        # 资源文件
└── data/                 # 测试数据
```

### 三层架构

```
┌────────────────────────────────────────┐
│ Level 3: 应用层 (cloudtool/)            │
│ MainWindow, Tools, Plugins            │
└──────────────┬─────────────────────────┘
               │
┌──────────────┴─────────────────────────┐
│ Level 2: UI 组件层 (widgets/)          │
│ CustomDialog, CloudTree, 对话框        │
└──────────────┬─────────────────────────┘
               │
┌──────────────┴─────────────────────────┐
│ Level 1: 核心与算法层                  │
│ ct_core: Cloud, Octree, FileIO, View   │
│ ct_modules: Filters, Features, Reg     │
└────────────────────────────────────────┘
```

## 支持的文件格式

| 格式 | 扩展名 | 读取 | 写入 | 说明 |
|------|--------|------|------|------|
| LAS | .las | √ | √ | ASPRS LAS 格式 |
| LAZ | .laz | √ | √ | 压缩 LAS 格式 |
| PLY | .ply | √ | √ | Stanford PLY 格式 |
| PCD | .pcd | √ | √ | PCL 点云格式 |
| TXT | .txt | √ | √ | 文本格式（支持字段映射） |
| OBJ | .obj | √ | - | Wavefront OBJ 格式 |

## 主要功能模块

### 滤波 (Filters)
- PassThrough - 直通滤波
- VoxelGrid - 体素降采样
- StatisticalOutlierRemoval - 统计离群点移除
- RadiusOutlierRemoval - 半径离群点移除
- ConditionalRemoval - 条件滤波
- GridMinimum - 2D 网格最小值投影
- LocalMaximum - 局部最大值移除

### 配准 (Registration)
- ICP - 迭代最近点算法
- IA-RANSAC - 基于特征的初始配准
- NDT - 正态分布变换

### 特征提取 (Features)
- PFH - 点特征直方图
- FPFH - 快速点特征直方图
- VFH - 视点特征直方图
- SHOT - 3D 形状描述符
- ESF - 形状函数集合

### 插件系统
- **CSF Plugin** - 地面点分割
- **Vegetation Plugin** - 植被分割 (支持 Otsu 自动阈值)
- **Change Detect Plugin** - 变化检测

## 技术亮点

### AOS 内存布局
采用 Array of Structures (AOS) 格式存储点云数据，相比 SOA 结构节省内存，可处理更大规模的点云文件。

### 八叉树 + LOD
- 空间分割递归划分 3D 空间
- 叶子节点最多存储 6 万个点
- 蓄水池采样 (Reservoir Sampling) 实时生成 LOD

### SSE 遍历策略
根据屏幕投影误差 (Screen Space Error) 动态选择渲染精细度，交互时降低质量，静止时提高质量。

### 全局坐标偏移
渲染前减去质心，保存时加回，解决 UTM 大坐标导致的 GPU 精度问题。

## Git 提交规范

遵循约定式提交格式：

```
<type>(<scope>): <description>
```

**类型 (type):**
- `feat` - 新功能
- `fix` - 缺陷修复
- `docs` - 文档更新
- `style` - 代码格式
- `refactor` - 代码重构
- `perf` - 性能优化
- `test` - 测试相关
- `chore` - 构建/工具

**示例:**
```
feat(ui): 添加导出按钮
fix(core): 修复空指针崩溃
refactor(render): 优化八叉树渲染性能
```

## 第三方库

### LAStools
- **功能**: LAS/LAZ 格式支持
- **许可**: 详见 3rdparty/LAStools/LICENSE
- **配置**: 通过 `add_subdirectory()` 调用其 CMakeLists

### CSF (Cloth Simulation Filter)
- **功能**: 地面点分割的布料模拟滤波器
- **论文**: [Fast and Robust Ground Filter](https://www.mdpi.com/2072-4292/8/6/506)
- **配置**: 使用自定义 CMakeLists 配合 `file(GLOB)`

## 预处理器定义

| 宏 | 值 | 说明 |
|----|----|----|
| `ROOT_PATH` | 项目根目录 | 项目源码路径 |
| `DATA_PATH` | data/ | 数据文件路径 |
| `CT_LIBRARY` | - | 标记库导出 |

## 常见问题

### Q: 编译时找不到 Qt5/VTK/PCL？
A: 请确保已正确设置 `CMAKE_PREFIX_PATH` 或环境变量。

### Q: 大点云文件打开缓慢？
A: 已启用流式加载和 LOD 渲染，可尝试调整 `CloudConfig` 中的参数。

### Q: UTM 坐标渲染时抖动？
A: 使用全局坐标偏移功能，在导入时设置合适的偏移值。

## 许可证

本项目使用的第三方库请遵循各自的许可证协议。

## 贡献

欢迎提交 Issue 和 Pull Request！

---

**CloudTool2** - 专业的三维点云处理解决方案
