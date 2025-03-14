# 一、PCL相关
## 1.1 点云的定义
```cpp
// cloud1定义的是一个点云容器，存储了多个pcl::PointXYZRGBNormal类型的点
pcl::PointCloud<pcl::PointXYZRGBNormal> cloud1;
// cloud2定义的是一个单独的点，表示一个具体的点
pcl::PointXYZRGBNormal cloud2;
```
###  区别
`pcl::PointCloud` 是 PCL 中表示点云数据的容器类型，它是一个模板类，可以存储任意类型的点。
`pcl::PointCloud<pcl::PointXYZRGBNormal>` 表示一个点云，里面的每个点是 pcl::PointXYZRGBNormal 类型，
包含了点的位置（x, y, z），颜色（r, g, b），以及法线信息（normal_x, normal_y, normal_z）。
这意味着 `pcl::PointCloud<pcl::PointXYZRGBNormal>` 是一个点云数据结构，而 pcl::PointXYZRGBNormal 是点的类型。


