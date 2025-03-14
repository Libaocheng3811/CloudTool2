#ifndef CLOUDTOOL2_CLOUDVIEW_H
#define CLOUDTOOL2_CLOUDVIEW_H

#include "base/cloud.h"
#include "base/exports.h"

#include "QVTKOpenGLNativeWidget.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/range_image/range_image.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkOrientationMarkerWidget.h"

namespace ct {

    struct PointXY
    {
        // 带参构造函数，在构造对象时必须传入两个参数，
        PointXY(int x, int y) : x(x), y(y) {}
        bool operator==(const PointXY& pt) const
        {
            return (this->x == pt.x) && (this->y == pt.y);
        }
        bool operator!=(const PointXY& pt) const { return !(*this == pt); }
        // 既然已经使用了带参构造函数，那为什么还要给x,y赋值0.0f呢？？？
        float x = 0.0f;
        float y = 0.0f;
    };

    class CT_EXPORT CloudView : public QVTKOpenGLNativeWidget{
        Q_OBJECT
    public:
        // 构造函数前的 explicit 关键字用于防止某些类型的隐式转换和复制构造函数的生成
        // 默认构造函数，explicit声明防止了编译器对某些类型的隐式类型转换.
        explicit CloudView(QWidget* parent = nullptr);

        ////////////////////////////////////////////////////////
        // add
        /**
         * @brief 添加点云
         */
        void addPointCloud(const Cloud::Ptr& cloud);

        /**
         * @brief 从深度图中添加点云
         */
        void addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString& id, const RGB& rgb = Color::White);

        /**
         * @brief 添加点云包围盒
         */
        void addBox(const Cloud::Ptr& cloud);

        /**
         * @brief 添加点云法线
         */
        void addPointCloudNormals(const Cloud::Ptr& cloud, int level, float scale);

        /**
         * @brief 添加点对的对应关系
         */
        void addCorrespondences(const Cloud::Ptr& source_points, const Cloud::Ptr& target_points,
                                const pcl::CorrespondencesPtr& correspondences, const QString& id = "correspondences");

        /**
         * @brief 添加多边形
         */
        void addPolygon(const Cloud::Ptr& cloud,
                        const QString& id = "polygon", const RGB& rgb = Color::White);

        /**
         * @brief 添加箭头
         */
        void addArrow(const PointXYZRGBN& pt1, const PointXYZRGBN& pt2,
                      const QString& id = "arrow", bool display_length = false, const RGB& rgb = Color::White);

        /**
         * @brief 添加立方体
         */
        void addCube(const pcl::ModelCoefficients::Ptr& coefficients,
                     const QString& id = "cube");

        /**
         * @brief 添加立方体
         */
        void addCube(const PointXYZRGBN& min, PointXYZRGBN& max,
                     const QString& id = "cube", const RGB& rgb = Color::White);

        /**
         * @brief 添加立方体
         */
        void addCube(const Box& box, const QString& id = "cube");

        ////////////////////////////////////////////////////////
        // 2D->3D(display to world)

        /**
         * @brief 屏幕2D坐标映射为3D坐标  将二维显示坐标转换成三维世界坐标
         */
        PointXYZRGBN displayToWorld(const PointXY& xy);

        /**
         * @brief 添加相对屏幕的2D多边形
         */
        void addPolygon2D(const std::vector<PointXY>& points,
                          const QString& id = "polyline", const RGB& rgb = Color::White);


        // point pick

        /**
         * @brief 单点选择
         * @param p 屏幕2D坐标点
         * @return int 选中点云的点索引
         * 主要是实现鼠标点击操作选点的功能
         */
        int singlePick(const PointXY& p);

        /**
         * @brief 多边形选取,确定哪些点位于指定的多边形区域内或外
         * @param points 屏幕2D多边形顶点
         * @param cloud 选取的点云
         * @param in_out 选择是否反向
         * @return std::vector<int> 选中点云的点索引集合
         */
        std::vector<int> areaPick(const std::vector<PointXY>& points, const Cloud::Ptr& cloud, bool in_out = false);


        ///////////////////////////////////////////////////////
        // remove
        /**
         * @brief 移除点云
         */
        void removePointCloud(const QString& id);

        /**
         * @brief 移除模型
         */
        void removeShape(const QString& id);

        /**
         * @brief 移除对应关系
         */
        void removeCorrespondences(const QString& id);

        /**
         * @brief 移除所有点云
         */
        void removeAllPointClouds();

        /**
         * @brief 移除所有模型
         */
        void removeAllShapes();

        ///////////////////////////////////////////////////////////
        // properties
        /**
         * @brief 设置点云颜色 (RGB)
         */
        void setPointCloudColor(const Cloud::Ptr& cloud, const RGB& rgb = Color::White);

        /**
         * @brief 设置点云颜色 (RGB)
         */
        void setPointCloudColor(const QString& id, const RGB& rgb = Color::White);

        /**
         * @brief 设置点云颜色（维度） 根据指定的坐标轴为点云设置颜色
         * 如果选择 X 坐标作为颜色依据，那么点云中每个点的颜色会根据其 X 坐标的值改变。
         * 例如，在 X 坐标为负值的点可以显示为蓝色，而正值的点显示为红色
         */
        void setPointCloudColor(const Cloud::Ptr& cloud, const QString& axis);

        /**
         * @brief 重置点云颜色，将点云颜色重置成其默认的RGB颜色
         */
        void resetPointCloudColor(const Cloud::Ptr& cloud);

        /**
         * @brief 设置点云大小
         */
        void setPointCloudSize(const QString& id, float size);

        /**
         * @brief 设置点云透明度
         */
        void setPointCloudOpacity(const QString& id, float value);

        /**
         * @brief 设置背景颜色
         */
        void setBackgroundColor(const RGB& rgb = Color::White);

        /**
         * @brief 重置背景颜色
         */
        void resetBackgroundColor();

        /**
         * @brief 设置模型颜色
         */
        void setShapeColor(const QString& shapeid, const RGB& rgb = Color::White);

        /**
         * @brief 设置模型点大小
         */
        void setShapeSize(const QString& shapeid, float size);

        /**
         * @brief 设置模型透明度
         */
        void setShapeOpacity(const QString& shapeid, float value);

        /**
         * @brief 设置模型线宽
         */
        void setShapeLineWidth(const QString& shapeid, float value);

        /**
         * @brief 设置模型字体大小
         */
        void setShapeFontSize(const QString& shapeid, float value);

        /**
         * @brief 设置模型表示类型
         * @param type 0-点，1-线，2-面
         */
        void setShapeRepersentation(const QString& shapeid, int type);

        ///////////////////////////////////////////////////////////
        // camera
        /**
         * @brief 重置相机参数
         */
         void resetCamera()
        {
             // 调用PCLVisualizer类的resetCamera成员函数，它将视图器中的相机重置到默认的视角和位置。
             // 调用getRenderWindow函数来获取当前视图器的渲染窗口，并调用其Render成员函数，强制渲染窗口立即更新显示内容
             m_viewer->resetCamera();
             m_viewer->getRenderWindow()->Render();
        }

        ///////////////////////////////////////////////////////////////
        // display
        /**
         * @brief 显示视图器信息
         * @param level 信息位置1-10
         */
        void showInfo(const QString& text, int level, const RGB& rgb = Color::White);

        /**
         * @brief 清除视图器信息
         */
        void clearInfo();

        /**
         * @brief 显示点云ID
         */
        void showCloudId(const QString& id);

        /**
         * @brief 设置是否显示点云ID
         */
        void setShowId(const bool& enable);

        /**
         * @brief 设置是否显示帧率
         */
        void setShowFPS(const bool& enable)
        {
            m_viewer->setShowFPS(enable);
        }

        /**
         * @brief 设置是否显示坐标系小部件
         */
        void setShowAxes(const bool& enable)
        {
            m_axes->SetEnabled(enable);
            m_viewer->getRenderWindow()->Render();
        }

        ///////////////////////////////////////////////////////////
        // other
        /**
         * @brief 检查具有给定ID的点云、模型或坐标是否已添加到视图中
         */
        bool contains(const QString& id)
        {
            // 调用pcl::visualization::PCLVisualizer::Ptr类的contains方法，返回一个bool值，表示是否包含这个id
            return m_viewer->contains(id.toStdString());
        }

        /**
         * @brief 设置是否开启交互
         */
        void setInteractorEnable(const bool& enable);

        ////////////////////////////////////////////////////////
        // viewport
        /**
         * @brief 设置为俯视图
         */
        void setTopView()
        {
            // setCameraPosition: 这个函数是 PCLVisualizer 类的方法，用于设置相机的位置和方向
            // 第一个参数表示相机的位置坐标，第二个表示相机的目标位置，也就是相机观察的方向；第三个表示相机的“上方向”向量，定义相机的顶部方向，这通常用于确定相机的倾斜角度。
            // 上方向的主要作用是确定相机的姿态和旋转的参考，避免不必要的旋转或视图混乱。
            // 一个小技巧，可以用脑袋来理解这个过程，脑袋的位置是相机位置，视线朝向目标位置，头顶方向就是上方向
            m_viewer->setCameraPosition(0, 0, 0, 0, -1, 0, 0, 0, -1);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为正视图
         */
        void setFrontView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为左视图
         */
        void setLeftSideView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 1, 0, 0, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置后视图
         */
        void setBackView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置右视图
         */
        void setRightSideView()
        {
            m_viewer->setCameraPosition(0, 0, 0, -1, 0, 0, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置底视图
         */
        void setBottomView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 1, 0, 0, 0, 1);
            m_viewer->getRenderWindow()->Render();
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief 任何从QObject派生的类都可以重新实现event()函数，（一个类接收到事件后，首先会由函数event处理）
         * QWidget就是重写了event()函数，并针对典型事件定义了专门的事件处理函数
         */
        // 事件处理函数-----时间处理函数都是从Qt的基础类（例如 QWidget 或 QMainWindow）继承而来的。这些基础类提供了默认的事件处理机制。
        // 这里定义的事件处理函数，当对应的事件类型发生时，会执行对应的事件处理函数，比如你移动了鼠标，就发生了MouseMove事件（这是一个事件类型，不是事件类）
        // 然后就会对应执行mouseMoveEvent()这个事件处理函数
    protected:
        void mousePressEvent(QMouseEvent* event) override;
        void mouseReleaseEvent(QMouseEvent* event);
        void mouseMoveEvent(QMouseEvent* event);

    signals:
        void viewerPose(Eigen::Affine3f);
        void sizeChanged(const QSize& size);
        void posChanged(const QPoint& pos);
        void mouseLeftPressed(const PointXY& pt);
        void mouseLeftReleased(const PointXY& pt);
        void mouseRightPressed(const PointXY& pt);
        void mouseRightReleased(const PointXY& pt);
        void mouseMoved(const PointXY& pt);

    private:
        // 在 Qt 中，Q_DISABLE_COPY 宏用于阻止一个类的对象被复制。这个宏通过删除复制构造函数和赋值操作符来实现，使得类的对象不能被复制。
        Q_DISABLE_COPY(CloudView);
        // 是否在视图器中显示点云ID
        bool m_show_id;
        // 表示视图器中显示的信息的等级，也就是优先级
        int m_info_level;
        // 存储最后一次显示点云的ID
        QString m_last_id;

        // 创建了一个名为 m_viewer 的智能指针，该指针是 pcl::visualization::PCLVisualizer 类型, Ptr 是 PCLVisualizer 类的一个内部类模板
        // pcl是一个命名空间（namespace）,visualization也是一个命名空间，它是 pcl 命名空间下的子命名空间。它包含了PCL中与三维可视化相关的类和函数
        // PCLVisualizer是一个类名，属于 pcl::visualization 命名空间。PCLVisualizer 类是PCL库中用于创建和管理三维可视化窗口的类
        pcl::visualization::PCLVisualizer::Ptr m_viewer;

        // vtkSmartPointer是一个VTK库中定义的智能指针模板类。它用于自动管理VTK对象的内存
        // vtkRenderer 是VTK中的一个类，用于定义一个3D渲染场景
        // vtkGenericOpenGLRenderWindow 是 VTK 库中的一个类，它提供了一个框架，用于实现一个使用自己的 OpenGL 上下文和 drawable 的渲染窗口
        // vtkOrientationMarkerWidget 是 VTK 中用于创建和操作一个方向标记小部件的类
        vtkSmartPointer<vtkRenderer> m_render;
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
        vtkSmartPointer<vtkOrientationMarkerWidget> m_axes;

    };
} // namespace ct

#endif //CLOUDTOOL2_CLOUDVIEW_H
