#ifndef CLOUDTOOL2_CLOUD_H
#define CLOUDTOOL2_CLOUD_H

#include "base/exports.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/console/time.h"

#include <QFileInfo>
#include <QString>

// 定义一个宏。具体来说，CLOUD_TYPE_XYZ 是一个宏名称，而 "xyz" 是这个宏的值。
// 当编译器遇到 CLOUD_TYPE_XYZ 时，它会将其替换为字符串 "xyz"
#define CLOUD_TYPE_XYZ       "xyz"
#define CLOUD_TYPE_XYZRGB    "XYZRGB"
#define CLOUD_TYPE_XYZN      "XYZNormal"
#define CLOUD_TYPE_XYARGBN   "XYZRGBNormal"

#define BOX_PRE_FLAG         "-box"
#define NORMALS_PRE_FLAG     "-normals"

namespace ct
{
    // typedef 关键字用于为类型创建一个新的别名
    // pcl::Indices 是PCL中一个类，用于存储点云中点的索引
    typedef pcl::Indices Indices;
    // pcl::console::TicToc 是PCL中的一个工具类，它用于测量代码的执行时间
    typedef pcl::console::TicToc TicToc;
    // pcl::PointXYZRGBNormal是一个点云数据结构
    typedef pcl::PointXYZRGBNormal PointXYZRGBN;

    /* using和typedef
     * typedef 只能用于类型别名
     * using 关键字不仅可以用于为类型创建别名，还可以用于为模板、函数、命名空间等创建别名。
     */

    // 包围盒
    struct Box
    {
        double width;
        double height;
        double depth;

        // Eigen::Affine3f 是 Eigen 库中定义的一个仿射变换类，它表示一个 3D 空间中的仿射变换，可以包含旋转、平移和缩放。
        // Eigen::Vector3f 是 Eigen 库中用于表示 3D 空间中的向量的类模板，f 后缀表示向量中的元素是单精度浮点数（float 类型）
        // Eigen::Quaternionf 是 Eigen 库中用于表示四元数的类模板，f 后缀表示四元数中的元素是单精度浮点数（float 类型）
        // pose表示三维空间中的仿射变换，包括平移、旋转、缩放。
        Eigen::Affine3f pose;
        // translation表示平移，用于描述盒子经过平移后的位置
        Eigen::Vector3f translation;
        // rotation旋转，用于表示三维空间中的旋转
        Eigen::Quaternionf rotation;
    };

    struct RGB
    {
        // 默认构造函数
        RGB() {};
        // 带参构造函数
        RGB(int r_, int g_, int b_) :r(r_), g(g_), b(b_) {}
        double rf() const {return (double )r / 255; }
        double gf() const {return (double )g / 255; }
        double bf() const {return (double )b / 255; }
        int r;
        int g;
        int b;
    };

    // define color
    // 颜色命名空间
    namespace Color
    {
        const RGB White = {255, 255, 255};
        const RGB Black = {0, 0, 0};
        const RGB Red = {255, 0, 0};
        const RGB Green = { 0,  255,0 };
        const RGB Blue = { 0,  0,  255 };
        const RGB Yellow = { 255,255,0 };
        const RGB Cyan = { 0,255,255 };
        const RGB Purple = { 255,0,255 };
    }

    // 定义点云类，继承自pcl::PointCloud<PointXYZRGBN>
    // pcl::PointXYZRGBNormal 是一个点的数据结构，而 pcl::PointCloud<PointXYZRGBN> 是一个包含这种点的点云容器。
    class CT_EXPORT Cloud :public pcl::PointCloud<PointXYZRGBN>
    {
    public:
        // Cloud类的构造函数
        Cloud() : m_id("cloud"),
                  m_box_rgb(Color::White),
                  m_normals_rgb(Color::White),
                  m_type(CLOUD_TYPE_XYZ),
                  m_point_size(1),
                  m_opacity(1.0),
                  m_resolution(0.0)
        {}

        // 复制构造函数，但不是标准复制构造函数，它接受一个对 Cloud 类型对象的常量引用，表示要复制的点云。一个对 Indices 类型对象的常量引用，表示要选择的点的索引
        // 调用了基类 pcl::PointCloud 的复制构造函数，并传递了 cloud 和 indices 参数。
        // pcl::PointCloud<PointXYZRGBN> 是 PCL 中用于存储点云数据的类模板。
        // 模板参数 PointXYZRGBN 指定了点云中点的类型，它包含了点的三维坐标、颜色和法线信息
        Cloud(const Cloud& cloud, const Indices& indices) :pcl::PointCloud<PointXYZRGBN>(cloud, indices) {}

        /* 标准复制构造函数接受一个对当前类类型的常量引用作为唯一参数，形式：Cloud(const Cloud& other);
         * 然而，复制构造函数并不一定只能有一个参数。它也可以有其他参数，只要至少有一个参数是对同类型对象的常量引用
         */

        // 重载运算符，接受一个常量引用类型的 Cloud 对象作为参数，并返回对当前对象的引用
        // 在链式地使用 += 操作符时非常有用
        Cloud& operator+=(const Cloud& rhs)
        {
            // concatenate是PCL 库中的一个函数，用于将两个点云对象的数据合并。
            // this表示指向当前对象的指针，*this是对当前指针的解引用，表示当前对象本身
            concatenate((*this), rhs);
            // 返回对当前对象的引用， 就是将rhs合并到了当前点云中
            return (*this);
        }

        // 重载运算符，返回合并后的cloud对象
        Cloud operator+(const Cloud& rhs)
        {
            return (Cloud(*this) += rhs);
        }

        // 为智能指针类型创建别名, 简化代码
        // 创建了一个名为 ConstPtr 的别名，它代表一个指向 const Cloud 类型对象的 std::shared_ptr 智能指针
        // const Cloud 表示指向的对象不能被修改，即这个 shared_ptr 只能指向一个常量对象。
        using Ptr = std::shared_ptr<Cloud>;
        using ConstPtr = std::shared_ptr<const Cloud>;

        // new Cloud(*this)：使用当前对象的复制构造函数创建一个新的 Cloud 对象。
        // 这里调用的是默认的复制构造函数
        // 返回一个指向新创建的Cloud对象的共享指针
        Ptr makeShared() const
        {
            // this指向当前对象的地址，是一个指向对象的指针，类型为Cloud*
            // *this是进行解引用操作，表示当前对象的实例本身
            // 使用的是 Cloud 类的复制构造函数，它接受一个 Cloud 类型的对象作为参数
            return Ptr(new Cloud(*this));
        }

        /**
         * @brief 点云包围盒
         * @note const说明这个是常量成员函数，这个成员函数不会修改对象的任何成员变量
         */
        Box box() const {return m_box;}

        /**
         * @brief 点云ID
         */
        QString id() const {return m_id;}

        /**
         * @brief 点云法线ID
         */
        QString normalId() const {return m_id + NORMALS_PRE_FLAG;}

        /**
         * @brief 点云包围盒ID
         */
        QString boxId() const {return m_id + BOX_PRE_FLAG;}

        /**
         * @brief 点云包围盒颜色
         */
        RGB boxColor() const {return m_box_rgb;}

        /**
         * @brief 点云法线颜色
         */
        RGB normalColor() const {return m_normals_rgb;}

        /**
         * @brief 点云中心
         */
        Eigen::Vector3f center() const {return m_box.translation;}

        /**
         * @brief 点云类型
         */
        QString type() const {return m_type;}

        /**
         * @brief 点云文件信息
         */
        QFileInfo info() const {return m_info;}

        /**
         * @brief 点云文件路径
         */
        QString path() const {return m_info.path();}

        /**
         * @brief 点云最小点
         */
        PointXYZRGBN min() const {return m_min;}

        /**
         * @brief 点云最大点
         */
        PointXYZRGBN max() const {return m_max;}

        /**
         * @brief 点云文件大小(KB)
         */
        int fileSize() const {return m_info.size() / 1024;}

        /**
         * @brief 获取点云点大小
         */
        int pointSize() const {return m_point_size;}

        /**
         * @brief 点云透明度
         */
        float opacity() const {return m_opacity;}

        /**
         * @brief 点云分辨率
         */
        float resolution() const {return m_resolution;}

        /**
         * @brief 点云体积
         */
        float volume() const {return m_box.width * m_box.height * m_box.depth;}

        /**
         * @brief 点云是否有法线
         * @param points[...]：通过随机索引访问 points 容器中的一个点
         * normal_x != 0.0f：这部分检查选中点的法线在 x 方向上的分量是否不等于 0.0。如果 normal_x 不为 0.0，则认为该点具有法线信息
         * 为什么没有参数传递并且函数体中也没有显示的成员变量？----
         * -----c++中，类的成员函数可以直接访问对象的变量成员，point是Cloud类的成员变量，是从pcl::PointCloud<PointXYZRGBN>继承而来的
         */
        bool hasNormals() const {return points[rand() % size()].normal_x != 0.0f;}

        /**
         * @brief 设置点云ID
         */
        void setId(const QString& id) {m_id = id;}

        /**
         * @brief 设置点云包围盒
         */
        void setBox(const Box& box) {m_box = box;}

        /**
         * @brief 设置点云文件信息
         */
        void setInfo(const QFileInfo& info) {m_info = info;}

        /**
         * @brief 设置点云点大小
         */
        void setPointSize(int point_size) {m_point_size = point_size;}

        /**
         * @brief 设置点云点颜色(RGB)
         */
        void setCloudColor(const RGB& rgb);

        /**
         * @brief 设置点点颜色(aixs)
         */
        void setCloudColor(const QString& axis);

        /**
         * @brief 设置包围盒颜色
         */
        void setBoxColor(const RGB& rgb) {m_box_rgb = rgb;}

        /**
         * @brief 设置法线颜色
         */
        void setNormalColor(const RGB& rgb) {m_normals_rgb = rgb;}

        /**
         * @brief 设置点云透明度
         */
        void setOpacity(float opacity) {m_opacity = opacity;}

        /**
         * @brief 按照维度(x,y,z)缩放点云尺寸
         * @param[1] origin 是否以坐标原点为缩放中心
         */
        void scale(double x, double y, double z, bool origin = false);

        /**
         * @brief 更新点云
         * @param box_flag 是否更新包围盒
         * @param type_flag 是否更新点云类型
         * @param resolution_flag 是否更新分辨率
         */
        void update(bool box_flag = true, bool type_flag = false, bool resolution_flag = true);


    private:
        // m_box的初始化在update()函数中完成
        Box m_box;
        // m_id存储的是点云文件名信息
        QString m_id;
        RGB m_box_rgb;
        // 法线向量的颜色
        RGB m_normals_rgb;
        QString m_type;
        // m_info存储点云的路径信息
        QFileInfo m_info;
        int m_point_size;
        // 点云透明度
        float m_opacity;
        float m_resolution;
        PointXYZRGBN m_min;
        PointXYZRGBN m_max;
    };
}



#endif //CLOUDTOOL2_CLOUD_H
