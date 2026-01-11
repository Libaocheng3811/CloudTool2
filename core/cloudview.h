#ifndef CLOUDTOOL2_CLOUDVIEW_H
#define CLOUDTOOL2_CLOUDVIEW_H

#include "cloud.h"
#include "exports.h"

#include "QVTKOpenGLNativeWidget.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkSmartPointer.h>

#include <QMap>

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

        /**
         * @brief 添加 3D 文本标签 (带引线和背景框)
         * @param pos 3D坐标点
         * @param text 显示的文本
         * @param id 标签的ID
         * @param r, g, b 文本颜色 (0-1.0)
         */
        void add3DLabel(const PointXYZRGBN& pos, const QString& text, const QString& id,
                        double r = 1.0, double g = 1.0, double b = 0.0);

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
        int singlePick(const PointXY& p, const QString& target_cloud_id = "");

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
         * @brief 设置为俯视图(+z)
         */
        void setTopView();

        /**
        * @brief 设置底视图(-Z)
        */
        void setBottomView();

        /**
         * @brief 设置为正视图(front)
         */
        void setFrontView();

        /**
        * @brief 设置后视图(Back)
        */
        void setBackView();

        /**
         * @brief 设置为左视图(Left)
         */
        void setLeftSideView();

        /**
         * @brief 设置右视图(Right)
         */
        void setRightSideView();

        /**
         * @brief 设置交互模式(LOD切换)
         * @param activate true=正在交互(显示稀疏)， false=静止(显示稠密)
         */
        void setInteractiveMode(bool activate);

        /**
         * @brief 是否启用自动渲染
         * @param enable true=自动渲染，false=手动渲染
         */
        void setAutoRender(bool enable) { m_auto_render = enable; }

        /**
         * @brief 手动刷新渲染窗口
         */
        void refresh(){ m_viewer->getRenderWindow()->Render(); }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /**
         * @brief 任何从QObject派生的类都可以重新实现event()函数，（一个类接收到事件后，首先会由函数event处理）
         * QWidget就是重写了event()函数，并针对典型事件定义了专门的事件处理函数
         */
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
        void setView(const Eigen::Vector3f& direction, const Eigen::Vector3f& up);

    private:
        struct InfoData{
            QString text;
            RGB rgb;
        };
        // Key = Level (int), Value = InfoData
        QMap<int, InfoData> m_active_infos;

    private:
        Q_DISABLE_COPY(CloudView);
        bool m_show_id;
        int m_info_level;
        QString m_last_id;
        QString m_current_id; // 用于记录当前显示的ID字符串
        pcl::visualization::PCLVisualizer::Ptr m_viewer;
        vtkSmartPointer<vtkRenderer> m_render;
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
        vtkSmartPointer<vtkOrientationMarkerWidget> m_axes;

        // 维护一个正在显示的Cloud::Ptr列表，方便进行预览模式切换
        std::vector<Cloud::Ptr> m_visible_clouds;
        bool m_auto_render = true; //默认开启自动渲染

    };
} // namespace ct

#endif //CLOUDTOOL2_CLOUDVIEW_H
