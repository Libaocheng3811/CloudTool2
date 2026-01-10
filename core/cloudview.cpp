#include "cloudview.h"

#include <vtkAutoInit.h>
// VTK_MODULE_INIT 宏用于初始化 VTK 模块
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <pcl/geometry/planar_polygon.h>
#include <vtkAxesActor.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>

#include <QDropEvent>
#include <QMimeData>
#include <QUrl>

#include <cmath>
#define INFO_CLOUD_ID  "info_cloud_id"
#define INFO_TEXT      "info_text"


namespace ct
{
    namespace
    {
        class PCLDisableInteractorStyle : public vtkInteractorStyleTrackballCamera
        {
        public:
            static PCLDisableInteractorStyle* New();

            vtkTypeMacro(PCLDisableInteractorStyle, vtkInteractorStyleTrackballCamera);

            virtual void OnLeftButtonDown() override {}
            virtual void OnMiddleButtonDown() override {}
            virtual void OnRightButtonDown() override {}
            virtual void OnMouseWheelForward() override {}
            virtual void OnMouseWheelBackward() override {}
        };
        // 一个VTK宏，自动定义new函数的实现，用于简化对象创建
        vtkStandardNewMacro(PCLDisableInteractorStyle);
    } // namespace

    CloudView::CloudView(QWidget *parent)
        : QVTKOpenGLNativeWidget(parent),
        m_show_id(true),
        m_info_level(0),
        m_last_id(""),
        m_render(vtkSmartPointer<vtkRenderer>::New()),
        m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
        m_axes(vtkSmartPointer<vtkOrientationMarkerWidget>::New())
    {
        // 将一个渲染器 (vtkRenderer) 添加到渲染窗口 (vtkRenderWindow) 中
        m_renderwindow->AddRenderer(m_render);

        // 分配智能指针指向的对象为一个新构建的对象，接受四个参数，vtkRenderer渲染器对象、vtkRenderWindow渲染窗口对象，可视化窗口名称、bool值是否在启动时自动旋转
        m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));

        this->setRenderWindow(m_viewer->getRenderWindow());

        // 配置渲染器和交互器，将这两个组件关联起来，这样交互器就可以知道在哪个渲染窗口上监听和响应用户的输入事件
        m_viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
        // 设置渲染窗口背景颜色
        m_viewer->setBackgroundColor((double )150.0 / 255.0, (double )150.0 / 255.0, (double )150.0 / 255.0 );

        connect(this, &CloudView::sizeChanged, [this](QSize size){
            if (m_show_id && !m_current_id.isEmpty()) {
                // 重新计算位置并更新
                m_viewer->updateText(m_current_id.toStdString(),
                                     size.width() - m_current_id.length() * 6 - 20,
                                     size.height() - 25,
                                     12, 1, 1, 1, INFO_CLOUD_ID);
            }
        });

        connect(this, &CloudView::sizeChanged, [this](QSize size){
            // 遍历所有当前活跃的 Info
            QMap<int, InfoData>::iterator i;
            for (i = m_active_infos.begin(); i != m_active_infos.end(); ++i) {
                int level = i.key();
                const InfoData& data = i.value();
                std::string id = INFO_TEXT + std::to_string(level);

                // 重新计算 Y 轴位置 (this->height() 改为 size.height())
                int y_pos = size.height() - 25 * level;

                // 更新文本位置
                m_viewer->updateText(data.text.toStdString(),
                                     10, y_pos,
                                     12,
                                     data.rgb.rf(), data.rgb.gf(), data.rgb.bf(),
                                     id);
            }
        });

        // 创建和配置一个坐标轴（Axes）对象，通常用于在三维可视化场景中显示一个坐标系
        vtkSmartPointer<vtkAxesActor> actor =vtkSmartPointer<vtkAxesActor>::New();
        m_axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
        m_axes->SetOrientationMarker(actor);
        m_axes->SetInteractor(m_viewer->getRenderWindow()->GetInteractor());
        m_axes->SetViewport(0.9, 0, 1, 0.15);
        m_axes->SetEnabled(true);
        m_axes->InteractiveOn();
        m_axes->InteractiveOff();

        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloud(const Cloud::Ptr &cloud)
    {
        bool found = false;
        for (auto& c : m_visible_clouds){
            if (c->id() == cloud->id()) {
                found = true;
                break;
            }
        }
        if (!found) m_visible_clouds.push_back(cloud);

        // 如果是大型点云，自动生成预览点云，只生成一次
        if (cloud->size() > 30000000 && cloud->getPreviewCloud() == nullptr){
            cloud->generatePreview();
        }

        if (!m_viewer->contains(cloud->id().toStdString()))
            m_viewer->addPointCloud<PointXYZRGBN>(cloud, cloud->id().toStdString());
        else
        {
            pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb_handler(cloud);
            // 调用PCLVisualizer类的updatePointCloud函数，用于更新视图器m_viewer中显示的点云数据
            m_viewer->updatePointCloud<PointXYZRGBN>(cloud, rgb_handler, cloud->id().toStdString());
        }

        if (cloud->pointSize() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       cloud->pointSize(), cloud->id().toStdString());
        // 设置点云透明度
        if (cloud->opacity() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       cloud->opacity(), cloud->id().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString &id, const ct::RGB &rgb)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color(image, rgb.r, rgb.g, rgb.b);
        // 判断是否添加了该点云，如果未添加就将点云数据和颜色添加到视图器中，否则就更新视图器
        if (!m_viewer->contains(id.toStdString()))
        {
            m_viewer->addPointCloud(image, range_image_color, id.toStdString());
        }
        else
            m_viewer->updatePointCloud(image, range_image_color, id.toStdString());
        // 刷新窗口
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addBox(const Cloud::Ptr& cloud)
    {
        std::string id = cloud->boxId().toStdString();
        if (!m_viewer->contains(id))
        {
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, cloud->boxId().toStdString());

            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                  pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                  id);
        }
        else
        {}

        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->boxColor().rf(),
                                              cloud->boxColor().gf(), cloud->boxColor().bf(), id);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudNormals(const Cloud::Ptr &cloud, int level, float scale)
    {
        // 如果视图器中没有这个点云id的法线，就添加法线;
        if (!m_viewer->contains(cloud->normalId().toStdString()))
            m_viewer->addPointCloudNormals<PointXYZRGBN >(cloud, level, scale, cloud->normalId().toStdString());
        else
        {
            m_viewer->removePointCloud(cloud->normalId().toStdString());
            m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normalId().toStdString());
        }
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                   cloud->normalColor().rf(), cloud->normalColor().gf(),
                                                   cloud->normalColor().bf(), cloud->normalId().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCorrespondences(const Cloud::Ptr &source_points, const Cloud::Ptr &target_points,
                                       const pcl::CorrespondencesPtr &correspondences, const QString &id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        else
            m_viewer->updateCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, const ct::RGB &rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addArrow(const ct::PointXYZRGBN &pt1, const ct::PointXYZRGBN &pt2, const QString &id, bool display_length, const ct::RGB &rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::addCube(const pcl::ModelCoefficients::Ptr &coefficients, const QString &id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(*coefficients, id.toStdString());
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const ct::PointXYZRGBN &min, ct::PointXYZRGBN &max, const QString &id, const ct::RGB &rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const ct::Box &box, const QString &id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, id.toStdString());
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    PointXYZRGBN CloudView::displayToWorld(const PointXY &pos)
    {
        double point[4];
        m_render->SetDisplayPoint(pos.x, pos.y, 0.1);
        m_render->DisplayToWorld();
        m_render->GetWorldPoint(point);
        return PointXYZRGBN(point[0], point[1], point[2], 0, 0, 0);
    }

    void CloudView::addPolygon2D(const std::vector<PointXY> &points, const QString &id, const ct::RGB &rgb)
    {
        Cloud::Ptr cloud(new Cloud);
        // 循环遍历将二维点转换成三维点
        for (auto& i : points)
        {
            PointXYZRGBN point = this->displayToWorld(i);
            // 将转换后的三维点存储在cloud中
            cloud->push_back(point);
        }
        // 添加多边形
        this->addPolygon(cloud, id, rgb);
    }

    // point pick
    int CloudView::singlePick(const ct::PointXY &pos)
    {
        vtkSmartPointer<vtkPointPicker> m_point_picker = vtkSmartPointer<vtkPointPicker>::New();
        m_renderwindow->GetInteractor()->SetPicker(m_point_picker);
        if (!m_point_picker)
            return -1;

        m_renderwindow->GetInteractor()->StartPickCallback();

        vtkRenderer* ren = this->GetInteractor()->FindPokedRenderer(pos.x, pos.y);
        /**
         * @brief 拾取三维场景中的点
         * @param Pick 是 vtkPointPicker 的核心方法，用于执行点的拾取操作。
         * @param pos.x pos.y 表示鼠标点击的屏幕坐标
         * @param 0.0 表示拾取的深度值
         * @param ren 鼠标点击位置对应的渲染器对象
         */
        m_point_picker->Pick(pos.x, pos.y, 0.0, ren);

        return (static_cast<int>(m_point_picker->GetPointId()));
    }

    std::vector<int> CloudView::areaPick(const std::vector<PointXY> &points, const Cloud::Ptr &cloud, bool in_out)
    {
        // 获取传入点集合的大小
        int size = points.size();
        float constant[99], multiple[99];
        int i, j = size - 1;
        for (i = 0; i < size; i++)
        {
            if (points[j].y == points[i].y)
            {
                constant[i] = points[i].x;
                multiple[i] = 0;
            }
            else
            {
                constant[i] = points[i].x - (points[i].y * points[j].x) / (points[j].y - points[i].y) +
                              (points[i].y * points[i].x) / (points[j].y - points[i].y);
                multiple[i] = (points[j].x - points[i].x) / (points[j].y - points[i].y);
            }
            j = i;
        }

        std::vector<int> indices;
        for (size_t i = 0; i < cloud->size(); i++)
        {
            m_render->SetWorldPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1);
            m_render->WorldToDisplay();
            double p[3];
            m_render->GetDisplayPoint(p);

            bool oddNodes = in_out, current = points[size - 1].y > p[1], previous;
            for (int m = 0; m < size; m++)
            {
                previous = current;
                current = points[m].y > p[1];
                if (current != previous)
                    oddNodes ^= p[1] * multiple[m] + constant[m] < p[0];
            }
            if (oddNodes) indices.push_back(i);
        }
        return indices;
    }

    ///////////////////////////////////////////////////////////////////
    // remove
    void CloudView::removePointCloud(const QString &id)
    {
        auto it = std::remove_if(m_visible_clouds.begin(), m_visible_clouds.end(),
                                 [&](const Cloud::Ptr& cloud) { return cloud->id() == id; });
        m_visible_clouds.erase(it, m_visible_clouds.end());

        std::string preview_id = id.toStdString() + "_preview";
        if (m_viewer->contains(preview_id))
            m_viewer->removePointCloud(preview_id);

        // 移除点云数据，并重新渲染窗口
        m_viewer->removePointCloud(id.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeShape(const QString& id)
    {
        std::string std_id = id.toStdString();
        // 移除包围盒模型
        if (m_viewer->contains(std_id)){
            m_viewer->removeShape(std_id);
            if (m_auto_render) m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::removeCorrespondences(const QString &id)
    {
        m_viewer->removeCorrespondences(id.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllPointClouds()
    {
        m_visible_clouds.clear();

        m_viewer->removeAllPointClouds();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllShapes()
    {
        m_viewer->removeAllShapes();
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const RGB& rgb)
    {
        // 使用 pcl::visualization::PointCloudColorHandlerCustom 创建一个颜色处理器，
        pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud, rgb.r, rgb.g, rgb.b);
        // updatePointCloud 方法用于更新可视化窗口中显示的点云。
        m_viewer->updatePointCloud(cloud, color, cloud->id().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const QString &id, const RGB &rgb)
    {
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString()
                );
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const QString& axis)
    {
        // 使用了 pcl::visualization::PointCloudColorHandlerGenericField 类，这个类可以基于点云中的特定字段（如某个坐标轴的值）创建颜色处理器。
        pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN>
                fieldcolor(cloud, axis.toStdString());
        m_viewer->updatePointCloud(cloud, fieldcolor, cloud->id().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetPointCloudColor(const Cloud::Ptr &cloud)
    {
        cloud->restoreColors();

        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb_handler(cloud);
        m_viewer->updatePointCloud(cloud, rgb_handler, cloud->id().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudSize(const QString &id, float size)
    {
        // setPointCloudRenderingProperties是PCLVisualizer类的一个成员函数，用于设置点云的渲染属性。size表示点的大小，id是点云标识符，0是视口索引
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.toStdString(), 0);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudOpacity(const QString &id, float value)
    {
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, id.toStdString(), 0);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setBackgroundColor(const ct::RGB &rgb)
    {
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        m_viewer->setBackgroundColor((double)150.0 / 255.0, (double)150.0 / 255.0, (double)150.0 / 255.0);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeColor(const QString &shapeid, const RGB &rgb)
    {
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                              rgb.rf(), rgb.gf(), rgb.bf(), shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeSize(const QString& shapeid, float size)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeOpacity(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeLineWidth(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeFontSize(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_FONT_SIZE, value, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeRepersentation(const QString &shapeid, int type)
    {
        // 设置模型表示类型
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type, shapeid.toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showInfo(const QString &text, int level, const RGB &rgb)
    {
        // 比较获取优先级，维护最大的信息级别
        m_info_level = std::max(m_info_level, level);
        m_active_infos[level] = {text, rgb};

        std::string id = INFO_TEXT + std::to_string(level);
        // 设置视图器中的显示信息
        if (!m_viewer->contains(id))
            m_viewer->addText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        else
            m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        // 当视图器大小改变时，同步改变信息位置

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::clearInfo()
    {
        for (int i = 0; i < m_info_level; i++)
        {
            std::string id = INFO_TEXT + std::to_string(i + 1);
            if (m_viewer->contains(id))
            {
                m_viewer->removeShape(id);
            }
        }
        m_active_infos.clear();
        m_info_level = 0;
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showCloudId (const QString& id)
    {
        m_last_id = id;
        m_current_id = id;
        if (!m_show_id)
        {
            return;
        }
        // 如果m_viewer不包含ID为INFO_CLOUD_ID的文本,使用addText函数将点云ID作为文本添加到可视化窗口中,如果存在就更新显示文本
        if (!m_viewer->contains(INFO_CLOUD_ID))
            m_viewer->addText(id.toStdString(), this->width() - id.length() * 6 - 20, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);
        else
            m_viewer->updateText(id.toStdString(), this->width() - id.length() *6 - 20, this->height() -25, 12, 1, 1, 1, INFO_CLOUD_ID);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShowId(const bool &enable)
    {
        m_show_id = enable;
        if (enable)
            showCloudId(m_last_id);
        else
            // 调用m_viewer的removeShape函数来移除显示的形状
            m_viewer->removeShape(INFO_CLOUD_ID);
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setInteractorEnable(const bool &enable)
    {
        /**
         * @brief vtkNew<> 是 VTK中提供的一个模板类，用于简化对象的创建和管理
         * 提供一个智能指针的便利性来管理 VTK 对象的生命周期，并确保在不再需要时释放占用的内存。
         */
        // 禁用交互器
        if (!enable)
        {
            // 创建一个PCLDisableInteractorStyle类对象style，
            vtkNew<PCLDisableInteractorStyle> style;
            // 将当前渲染窗口的交互器对象样式设置为style，从而实现禁用交互器
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        // 启用交互器
        else
        {
            vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    ///////////////////////////////////////////////////////////////////
    // viewport
    void CloudView::setView(const Eigen::Vector3f& direction, const Eigen::Vector3f& up) {
        m_viewer->resetCamera();

        m_viewer->getRenderWindow()->Render();
        vtkCamera* cam = m_render->GetActiveCamera();

        double* fp = cam->GetFocalPoint();

        double* pos = cam->GetPosition();
        double dist = std::sqrt(std::pow(pos[0] - fp[0], 2) +
                                std::pow(pos[1] - fp[1], 2) +
                                std::pow(pos[2] - fp[2], 2));

        double new_x = fp[0] + direction.x () * dist;
        double new_y = fp[1] + direction.y () * dist;
        double new_z = fp[2] + direction.z () * dist;

        m_viewer->setCameraPosition(
                new_x, new_y, new_z, // Eye,相机位置
                fp[0], fp[1], fp[2], // Target (焦点/看向哪里)
                up.x(), up.y(), up.z() // Up (头顶朝向)
                );

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setTopView() {
        // 俯视图：相机在 +Z，看向中心，头顶朝 +Y
        setView(Eigen::Vector3f(0, 0, 1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setBottomView() {
        // 底视图：相机在 -Z，看向中心，头顶朝 +Y
        setView(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0, 1, 0));
    }

    void CloudView::setFrontView() {
        // 正视图：通常定义为从 -Y 看向 +Y (或者从 +Y 看向 -Y，取决于你的坐标系习惯)
        // 这里假设 Z 是高，Y 是深。从 -Y 处看过去。头顶朝 +Z。
        setView(Eigen::Vector3f(0, -1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setBackView() {
        // 后视图：相机在 +Y，头顶朝 +Z
        setView(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setLeftSideView() {
        // 左视图：相机在 -X，头顶朝 +Z
        setView(Eigen::Vector3f(-1, 0, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setRightSideView() {
        // 右视图：相机在 +X，头顶朝 +Z
        setView(Eigen::Vector3f(1, 0, 0), Eigen::Vector3f(0, 0, 1));
    }

    void CloudView::setInteractiveMode(bool activate) {
        bool need_render = false;

        for (const auto& cloud : m_visible_clouds){
            //如果点云很小，不进行任何操作
            if (cloud->getPreviewCloud() == nullptr) continue;

            std::string full_id = cloud->id().toStdString();
            std::string prev_id = cloud->getPreviewCloud()->id().toStdString();

            if (activate){
                // 动态模式，显示预览点云，隐藏原始点云
                m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, full_id);
                if (!m_viewer->contains(prev_id)){
                    //如果是第一次显示预览，需要addPointCloud
                    pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb_handler(cloud->getPreviewCloud());
                    m_viewer->addPointCloud<PointXYZRGBN>(cloud->getPreviewCloud(), rgb_handler, prev_id);

//                    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING,
//                                                               pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, prev_id);

                    //同步点的大小
                    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                               cloud->pointSize(), prev_id);
                }
                else {
                    //如果已经存在，设置为可见
                    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, prev_id);
                }
            }
            else{
                //静态模式，隐藏预览点云，显示原始点云
                if (m_viewer->contains(prev_id)){
                    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, prev_id);
                }
                m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud->opacity(), full_id);
            }
            need_render = true;
        }
        if (need_render && m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::mousePressEvent(QMouseEvent *event)
    {
        setInteractiveMode(true);
        // button()是QMouseEvent的一个成员函数，返回一个 Qt::MouseButton 枚举值，表示哪个按键触发的事件
        if (event->button() == Qt::LeftButton)
        {
            emit mouseLeftPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                          m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                           m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        return QVTKOpenGLNativeWidget::mousePressEvent(event);
    }

    void CloudView::mouseReleaseEvent(QMouseEvent *event)
    {
        setInteractiveMode(false);

        if (event->button() == Qt::LeftButton)
        {
            // getViewerPose() 是 PCLVisualizer 类中的一个成员函数。它返回当前视图器的视角或位置的描述，例如相机的位置和方向。
            emit viewerPose(m_viewer->getViewerPose());
            emit mouseLeftReleased(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                           m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightReleased(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                            m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    }

    void CloudView::mouseMoveEvent(QMouseEvent *event)
    {
        emit mouseMoved(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        return QVTKOpenGLNativeWidget::mouseMoveEvent(event);
    }
}