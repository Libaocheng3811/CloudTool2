# 一、环境配置
UI框架：Qt5.15.2 + 渲染库VTK9.1.0 + 点云处理库PCL1.12.1

# 二、项目结构
- 主界面类：Mainwindow
- base文件夹：主界面逻辑处理类，UI控件类，日志输出类
- cloudtool/edit文件夹：点云编辑类，点云颜色设置，包围盒绘制
- cloudtool/resources文件夹：资源文件
- cloudtool/tool文件夹：点云处理类，已实现的有点云裁剪，点云滤波，点云选点，生成深度图
- moudles文件夹：点云处理模块类，封装PCL库函数
- data文件夹：点云数据