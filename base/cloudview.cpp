#include "base/cloudview.h"

#include <vtkAutoInit.h>
// VTK_MODULE_INIT 宏用于初始化 VTK 模块
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include "pcl/geometry/planar_polygon.h"
#include "vtkAxesActor.h"
#include "vtkPointPicker.h"

#include "QDropEvent"
#include "QMimeData"
#include "QUrl"

#define INFO_CLOUD_ID  "info_cloud_id"
#define INFO_TEXT      "info_text"


namespace ct
{
    // 定义一个匿名命名空间，匿名命名空间中的内容仅在本源文件中可见，避免了命名冲突
    namespace
    {
        // vtkInteractorStyleTrackballCamera 是一种支持使用鼠标进行相机操作的交互样式
        class PCLDisableInteractorStyle : public vtkInteractorStyleTrackballCamera
        {
        public:
            // 声明一个静态成员函数new，通常用来创建类的实例，并返回一个指向该实例的指针
            static PCLDisableInteractorStyle* New();

            // 是用VTK提供的宏vtkTypeMacro来为类添加类型信息，使得 VTK 能够在运行时获取该类的相关信息
            vtkTypeMacro(PCLDisableInteractorStyle, vtkInteractorStyleTrackballCamera);

            // 重写了父类中的方法，方法体为空，目的是禁用默认的相机交互行为
            virtual void OnLeftButtonDown() override {}
            virtual void OnMiddleButtonDown() override {}
            virtual void OnRightButtonDown() override {}
            virtual void OnMouseWheelForward() override {}
            virtual void OnMouseWheelBackward() override {}
        };
        // 一个VTK宏，自动定义new函数的实现，用于简化对象创建
        vtkStandardNewMacro(PCLDisableInteractorStyle);
    } // namespace

    // QVTKOpenGLNativeWidget(parent)调用基类 QVTKOpenGLNativeWidget 的构造函数，并将 parent 参数传递给它。这样做是为了正确地初始化基类部分
    // 实际上CloudView构造函数的QWidget指针parent就是给它的基类QVTKOpenGLNativeWidget准备的，用于正确初始化它的基类
    // 之所以是QWidget类的指针，是因为所有界面类的基类是QWidget类，这就正确指定了父子层次结构
    CloudView::CloudView(QWidget *parent)
        : QVTKOpenGLNativeWidget(parent),
        m_show_id(true),
        m_info_level(0),
        m_last_id(""),
        // 调用了 vtkSmartPointer<vtkRenderer> 的 New() 方法来创建一个新的 vtkRenderer 实例
        m_render(vtkSmartPointer<vtkRenderer>::New()),
        m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
        m_axes(vtkSmartPointer<vtkOrientationMarkerWidget>::New())
    {
        // 将一个渲染器 (vtkRenderer) 添加到渲染窗口 (vtkRenderWindow) 中
        m_renderwindow->AddRenderer(m_render);

        // 分配智能指针指向的对象为一个新构建的对象，接受四个参数，vtkRenderer渲染器对象、vtkRenderWindow渲染窗口对象，可视化窗口名称、bool值是否在启动时自动旋转
        m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));

        // 设置当前窗体CloudView的渲染窗口为m_viewer的渲染窗口，getRenderWindow()返回一个指向 vtkRenderWindow 对象的指针
        // 将 PCLVisualizer 对象的渲染窗口 (vtkRenderWindow) 设置给当前窗口。通常是为了让另一个对象能够与 PCLVisualizer 共享或使用同一个渲染窗口。
        this->setRenderWindow(m_viewer->getRenderWindow());

        // 配置渲染器和交互器，将这两个组件关联起来，这样交互器就可以知道在哪个渲染窗口上监听和响应用户的输入事件
        m_viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
        // 设置渲染窗口背景颜色
        m_viewer->setBackgroundColor((double )150.0 / 255.0, (double )150.0 / 255.0, (double )150.0 / 255.0 );

        // 创建和配置一个坐标轴（Axes）对象，通常用于在三维可视化场景中显示一个坐标系
        vtkSmartPointer<vtkAxesActor> actor =vtkSmartPointer<vtkAxesActor>::New();
        m_axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
        m_axes->SetOrientationMarker(actor);
        m_axes->SetInteractor(m_viewer->getRenderWindow()->GetInteractor());
        m_axes->SetViewport(0.9, 0, 1, 0.15);
        m_axes->SetEnabled(true);
        m_axes->InteractiveOn();
        m_axes->InteractiveOff();

        // 触发渲染窗口的渲染过程
        // pcl的PCLVisualizer类指针调用getRenderWindow()成员函数获得 vtkRenderWindow 对象的指针，
        // vtkRenderWindow 对象的指针调用Render成员函数触发渲染。
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloud(const Cloud::Ptr &cloud)
    {
        // 如果当前视图器对象中没有与要添加的点云id重复的点云，直接将点云添加到视图器中
        if (!m_viewer->contains(cloud->id().toStdString()))
            // 将cloud点云添加到m_viewer视图器对象中，addPointCloud函数是pcl::visualization::PCLVisualizer类的成员函数，用于将点云数据添加到可视化窗口中
            // cloud->id().toStdString()获取点云对象的ID，这个ID通常用于标识点云，以便在视图器中区分不同的点云
            m_viewer->addPointCloud<PointXYZRGBN>(cloud, cloud->id().toStdString());
        // ?
        else
        {
            // 创建了一个PointCloudColorHandlerRGBField对象rgb，它是用于处理点云颜色的。
            // PointXYZRGBN是一种点云类型，它包含了RGB颜色信息和法线（N）信息。
            pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
            // 调用PCLVisualizer类的updatePointCloud函数，用于更新视图器m_viewer中显示的点云数据
            m_viewer->updatePointCloud<PointXYZRGBN>(cloud, cloud->id().toStdString());
        }

        // 设置点云渲染属性，点的大小等
        // 检查点云中点是否有默认大小(默认大小为1)，
        if (cloud->pointSize() != 1)
            // 如果点云中点的大小不是默认1，则设置视图器中的显示的点云的大小为点云的实际大小
            // pcl::visualization::PCL_VISUALIZER_POINT_SIZE是一个枚举值，表示要设置的属性是点的大小。
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       cloud->pointSize(), cloud->id().toStdString());
        // 设置点云透明度
        if (cloud->opacity() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       cloud->opacity(), cloud->id().toStdString());
        // 用于强制渲染窗口,
        // m_viewer是一个指向pcl::visualization::PCLVisualizer类的智能指针，它是PCL中用于显示点云数据的视图器对象。
        // getRenderWindow()是PCLVisualizer类的成员函数，返回一个指向渲染窗口的指针。这个渲染窗口是负责绘制点云和其他可视化元素的OpenGL窗口
        // Render()：这是渲染窗口对象的一个成员函数，它告诉窗口立即进行渲染操作，将缓冲区中的内容绘制到屏幕上。
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudFromRangeImage(const pcl::RangeImage::Ptr &image, const QString &id, const ct::RGB &rgb)
    {
        // pcl::RangeImage 是一种存储深度图像的类，
        // pcl::visualization::PointCloudColorHandlerCustom 是 PCL 中用于设置点云颜色的一个类。它允许为点云中的每个点指定自定义的颜色。
        // range_image_color 是一个颜色处理器，将传入的 RGB 颜色值应用到 image（即 pcl::RangeImage）中的每个点。
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color(image, rgb.r, rgb.g, rgb.b);
        // 判断是否添加了该点云，如果未添加就将点云数据和颜色添加到视图器中，否则就更新视图器
        if (!m_viewer->contains(id.toStdString()))
        {
            m_viewer->addPointCloud(image, range_image_color, id.toStdString());
        }
        else
            m_viewer->updatePointCloud(image, range_image_color, id.toStdString());
        // 刷新窗口
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addBox(const Cloud::Ptr& cloud)
    {
        // contains函数是PCLVisualizer类的一个成员函数，它接受一个字符串参数，该字符串是图形对象的ID。
        // 检查视图器中是否包含了特定id的立方体
        if (!m_viewer->contains(cloud->boxId().toStdString()))
        {
            // 如果未包含立方体，则添加一个新的立方体
            // addCube是PCLVisualizer类的一个成员函数，用于在可视化窗口中添加一个立方体。
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, cloud->boxId().toStdString());
        }
        else
        {
            // 如果已经包含了该id的立方体，先移除旧的立方体，再添加一个新的立方体
            m_viewer->removeShape(cloud->boxId().toStdString());
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, cloud->boxId().toStdString());
        }
        // 设置立方体的渲染属性
        // PCL_VISUALIZER_REPRESENTATION：这是一个枚举值，表示要设置的渲染属性是形状的表示方式（Representation）
        // PCL_VISUALIZER_REPRESENTATION_WIREFRAME：这也是一个枚举值，指定形状的渲染方式为线框模式（Wireframe）。线框模式只渲染形状的边缘，不填充表面。
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                              cloud->boxId().toStdString());
        // 设置立方体的颜色
        // PCL_VISUALIZER_COLOR：这是一个枚举值，表示要设置的渲染属性是形状的颜色
        // cloud->boxColor().rf()调用Cloud类的boxColor函数获得包围盒颜色，再调用RGB结构体的rf()方法获得颜色分量的浮点值
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->boxColor().rf(),
                                              cloud->boxColor().gf(), cloud->boxColor().bf(), cloud->boxId().toStdString());
        // 强制渲染窗口，立即更新显示的内容
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudNormals(const Cloud::Ptr &cloud, int level, float scale)
    {
        // 如果视图器中没有这个点云id的法线，就添加法线;
        if (!m_viewer->contains(cloud->normalId().toStdString()))
            // addPointCloudNormals 是 PCLVisualizer 类的一个成员函数，用于在可视化窗口中添加点云的法线信息。
            // level：这是一个整数，表示法线的显示级别，通常用于控制法线的长度或显示的详细程度。
            // scale：这是一个浮点数，用于缩放法线的长度，使得法线在视觉上更易于观察。
            m_viewer->addPointCloudNormals<PointXYZRGBN >(cloud, level, scale, cloud->normalId().toStdString());
        else
        {
            m_viewer->removePointCloud(cloud->normalId().toStdString());
            m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normalId().toStdString());
        }
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                   cloud->normalColor().rf(), cloud->normalColor().gf(),
                                                   cloud->normalColor().bf(), cloud->normalId().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCorrespondences(const Cloud::Ptr &source_points, const Cloud::Ptr &target_points,
                                       const pcl::CorrespondencesPtr &correspondences, const QString &id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        else
            m_viewer->updateCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, const ct::RGB &rgb)
    {
        // 若多边形不存在，添加新的多边形
        if (!m_viewer->contains(id.toStdString()))
            // addPolygon是 PCLVisualizer 类中的一个模板函数，专门用于将多边形添加到视图器中
            // <PointXYZRGBN> 指定了多边形的顶点数据类型
            // cloud存储了构成多边形的点的信息，特别注意这里的cloud并不是我们一般说的某个点云数据，而是专门绘制多边形的点，比如四个点
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        // 若多边形已存在，删除旧的多边形，添加新的多边形
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addArrow(const ct::PointXYZRGBN &pt1, const ct::PointXYZRGBN &pt2, const QString &id, bool display_length, const ct::RGB &rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
            m_viewer->getRenderWindow()->Render();
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
        m_viewer->getRenderWindow()->Render();
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
        m_viewer->getRenderWindow()->Render();
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
        m_viewer->getRenderWindow()->Render();
    }

    PointXYZRGBN CloudView::displayToWorld(const PointXY &pos)
    {
        // 定义大小为4的数组point，存储三维世界坐标（x, y, z）及其齐次坐标值（通常是 w）
        double point[4];
        // VTK 渲染器m_render 调用 SetDisplayPoint 方法，将输入的显示坐标（pos 的 x 和 y 值）转换为 VTK 中的显示点
        m_render->SetDisplayPoint(pos.x, pos.y, 0.1);
        // 把之前的显示点转换为对应的世界坐标
        m_render->DisplayToWorld();
        // 获取世界点
        m_render->GetWorldPoint(point);
        // 以PCL中点的格式返回点
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
        /**
         * vtkSmartPointer 是 VTK 中提供的一种智能指针，用于管理 VTK 对象的生命周期。
         * vtkPointPicker 是 VTK 中的一个类，用于拾取三维场景中的点。
         * vtkPointPicker::New() 是一个工厂方法，用于创建一个新的 vtkPointPicker 实例。
         */
        // 创建一个指向 vtkPointPicker 对象的智能指针
        vtkSmartPointer<vtkPointPicker> m_point_picker = vtkSmartPointer<vtkPointPicker>::New();
        // 获取当前渲染窗口的交互器对象，并将新创建的点选择器设置为当前的选择器
        m_renderwindow->GetInteractor()->SetPicker(m_point_picker);
        if (!m_point_picker)
            return -1;
        /**
         * 交互器负责处理用户与渲染窗口的交互操作，例如鼠标点击、移动、键盘输入等。
         * 交互器对象的 StartPickCallback() 方法会启动拾取操作的回调函数。
         */
        // 开始选择过程，允许用户进行拾取操作
        m_renderwindow->GetInteractor()->StartPickCallback();
        /**
         * FindPokedRenderer() 是 vtkRenderWindowInteractor的方法，用于找到鼠标点击位置对应的渲染器
         * 在 VTK 中，一个渲染窗口可以包含多个渲染器（vtkRenderer），而每个渲染器可以渲染不同的内容或区域。
         * CloudView是一个包含渲染窗口的自定义类，m_renderwindow是该类中的一个渲染窗口，
         */
        // ren是一个渲染器
        vtkRenderer* ren = this->GetInteractor()->FindPokedRenderer(pos.x, pos.y);
        /**
         * @brief 拾取三维场景中的点
         * @param Pick 是 vtkPointPicker 的核心方法，用于执行点的拾取操作。
         * @param pos.x pos.y 表示鼠标点击的屏幕坐标
         * @param 0.0 表示拾取的深度值
         * @param ren 鼠标点击位置对应的渲染器对象
         */
        m_point_picker->Pick(pos.x, pos.y, 0.0, ren);
        /**
         * GetPointId() 是 vtkPointPicker 的方法，用于返回拾取到的点的 ID。
         * 如果拾取成功，它会返回点云中拾取点的索引 ID（从 0 开始编号）;拾取失败返回-1
         * static_cast<int>将返回值强制转换成int型
         */
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
        // 移除点云数据，并重新渲染窗口
        m_viewer->removePointCloud(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeShape(const QString& id)
    {
        // 移除包围盒模型
        m_viewer->removeShape(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeCorrespondences(const QString &id)
    {
        m_viewer->removeCorrespondences(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllPointClouds()
    {
        m_viewer->removeAllPointClouds();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllShapes()
    {
        m_viewer->removeAllShapes();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const RGB& rgb)
    {
        // 使用 pcl::visualization::PointCloudColorHandlerCustom 创建一个颜色处理器，
        pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud, rgb.r, rgb.g, rgb.b);
        // updatePointCloud 方法用于更新可视化窗口中显示的点云。
        m_viewer->updatePointCloud(cloud, color, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const QString &id, const RGB &rgb)
    {
        // setPointCloudRenderingProperties 是 PCLVisualizer 类的一个成员函数，用于设置渲染属性。
        // 此函数通过id来识别要设置的点云，如果传入的是点云ID，设置的就是点云颜色，如果传入的是法线ID，设置的就是法线颜色
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString()
                );
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, const QString& axis)
    {
        // 使用了 pcl::visualization::PointCloudColorHandlerGenericField 类，这个类可以基于点云中的特定字段（如某个坐标轴的值）创建颜色处理器。
        pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN>
                fieldcolor(cloud, axis.toStdString());
        m_viewer->updatePointCloud(cloud, fieldcolor, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetPointCloudColor(const Cloud::Ptr &cloud)
    {
        // 创建了一个颜色处理器rgb，它会根据cloud中存储的RGB信息生成颜色
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
        m_viewer->updatePointCloud(cloud, rgb, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudSize(const QString &id, float size)
    {
        // setPointCloudRenderingProperties是PCLVisualizer类的一个成员函数，用于设置点云的渲染属性。size表示点的大小，id是点云标识符，0是视口索引
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.toStdString(), 0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudOpacity(const QString &id, float value)
    {
        m_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, id.toStdString(), 0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setBackgroundColor(const ct::RGB &rgb)
    {
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        m_viewer->setBackgroundColor((double)150.0 / 255.0, (double)150.0 / 255.0, (double)150.0 / 255.0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeColor(const QString &shapeid, const RGB &rgb)
    {
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                              rgb.rf(), rgb.gf(), rgb.bf(), shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeSize(const QString& shapeid, float size)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeOpacity(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY, value, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeLineWidth(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeFontSize(const QString &shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_FONT_SIZE, value, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeRepersentation(const QString &shapeid, int type)
    {
        // 设置模型表示类型
        m_viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showInfo(const QString &text, int level, const RGB &rgb)
    {
        // 比较获取优先级，维护最大的信息级别
        m_info_level = std::max(m_info_level, level);
        std::string id = INFO_TEXT + std::to_string(level);
        // 设置视图器中的显示信息
        if (!m_viewer->contains(id))
            m_viewer->addText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        else
            m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        // 当视图器大小改变时，同步改变信息位置
        connect(this, &CloudView::sizeChanged, [=](QSize size)
                { m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id); });
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::clearInfo()
    {
        for (int i = 0; i < m_info_level; i++)
        {
            if (m_viewer->contains(INFO_TEXT + std::to_string(i + 1)))
            {
                m_viewer->removeShape(INFO_TEXT + std::to_string(i + 1));
            }
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showCloudId (const QString& id)
    {
        m_last_id = id;
        if (!m_show_id)
        {
            return;
        }
        // 如果m_viewer不包含ID为INFO_CLOUD_ID的文本,使用addText函数将点云ID作为文本添加到可视化窗口中,如果存在就更新显示文本
        if (!m_viewer->contains(INFO_CLOUD_ID))
            // addText函数中id.toStdString()才是实际显示的信息，而INFO_CLOUD_ID 只是用作这个文本对象的标识符
            // INFO_CLOUD_ID 的作用是为显示的文本提供一个唯一的标识，而它显示的内容则是根据函数参数 id 动态变化的
            m_viewer->addText(id.toStdString(), this->width() - id.length() * 6 - 20, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);
        else
            m_viewer->updateText(id.toStdString(), this->width() - id.length() *6 - 20, this->height() -25, 12, 1, 1, 1, INFO_CLOUD_ID);
        // 连接CloudView的sizeChanged信号到一个匿名函数（lambda表达式），当窗口大小发生变化时，将更新文本的位置
        // sizeChanged信号传递新的窗口大小QSize，然后使用这个新的大小来重新计算文本的位置，并调用updateText更新文本
        // 接收信号的对象实际上是 this，也就是 CloudView 实例本身，只是这个信息是通过 lambda 表达式的捕获列表隐式提供的，
        //  [=] 捕获了当前作用域内的所有变量，包括 this 指针，这意味着 lambda 表达式内部可以访问 this 指向的对象。
        connect(this, &CloudView::sizeChanged, [=](QSize size)
                {m_viewer->updateText(id.toStdString(), size.width() - id.length() * 6 - 20, size.height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID); });
        m_viewer->getRenderWindow()->Render();

    }

    void CloudView::setShowId(const bool &enable)
    {
        m_show_id = enable;
        if (enable)
            showCloudId(m_last_id);
        else
            // 调用m_viewer的removeShape函数来移除显示的形状
            m_viewer->removeShape(INFO_CLOUD_ID);
        m_viewer->getRenderWindow()->Render();
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
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::mousePressEvent(QMouseEvent *event)
    {
        // button()是QMouseEvent的一个成员函数，返回一个 Qt::MouseButton 枚举值，表示哪个按键触发的事件
        if (event->button() == Qt::LeftButton)
        {
            emit mouseLeftPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                          m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            // GetInteractor() 是一个方法，用于获取与渲染窗口关联的交互器对象（interactor）
            // GetEventPosition()是交互器的方法，返回一个整数数组，表示最近一次事件（在此例中是鼠标事件）发生时的屏幕坐标。
            // 数组中的第一个元素表示 x 坐标，第二个元素表示 y 坐标
            emit mouseRightPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                           m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        // 在自定义的小部件类（例如CloudView）中处理鼠标按下事件时，它允许基类QVTKOpenGLNativeWidget处理该事件。
        // 这对于确保事件能够继续传播并被适当地处理是很重要的
        return QVTKOpenGLNativeWidget::mousePressEvent(event);
    }

    void CloudView::mouseReleaseEvent(QMouseEvent *event)
    {
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