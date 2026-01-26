#include "cloudview.h"

#include <vtkAutoInit.h>
// VTK_MODULE_INIT 宏用于初始化 VTK 模块
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <vtkAxesActor.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>
#include <vtkProperty2D.h>
#include <vtkTextActor.h>

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
        m_renderwindow->AddRenderer(m_render);
        m_render->GetActiveCamera()->SetClippingRange(0.01, 10000.0);
        m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));
        this->setRenderWindow(m_renderwindow);

        m_viewer->setupInteractor(this->interactor(), this->renderWindow());
        m_render->GradientBackgroundOn();
        m_render->SetBackground2(0.05, 0.4, 0.6);
        m_render->SetBackground(0.00, 0.05, 0.08);

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
            QMap<int, InfoData>::iterator i;
            for (i = m_active_infos.begin(); i != m_active_infos.end(); ++i) {
                int level = i.key();
                const InfoData& data = i.value();
                std::string id = INFO_TEXT + std::to_string(level);

                int y_pos = size.height() - 25 * level;

                m_viewer->updateText(data.text.toStdString(),
                                     10, y_pos,
                                     12,
                                     data.rgb.rf(), data.rgb.gf(), data.rgb.bf(),
                                     id);
            }
        });

        vtkNew<vtkAxesActor> actor;
        m_axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
        m_axes->SetOrientationMarker(actor);

        m_axes->SetInteractor(this->interactor());
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
        auto renderCloud = cloud->getPreviewCloud();
        std::string cloudId = cloud->id().toStdString();

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(renderCloud);

        if (!m_viewer->contains(cloudId))
            m_viewer->addPointCloud<PointXYZRGB>(renderCloud, rgb_handler, cloudId);
        else
        {
            m_viewer->updatePointCloud<pcl::PointXYZRGB>(renderCloud, rgb_handler, cloudId);
        }

        if (cloud->pointSize() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       cloud->pointSize(), cloudId);
        // 设置点云透明度
        if (cloud->opacity() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       cloud->opacity(), cloudId);
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
        if (cloud->volume() <= 0.0f || cloud->box().width <= 0.0f){
            return;
        }
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
        // TODO 既然只显示预览点云，法线是不是也应该只显示预览点云的法线呢
        std::string id = cloud->normalId().toStdString();

        if (!cloud->hasNormals()) return;

        // 只显示预览点云的法线 (LOD)
        // 我们需要按同样的步长抽取法线
        auto previewXYZ = cloud->getPreviewCloud(); // PointXYZRGB
        size_t previewSize = previewXYZ->size();
        size_t fullSize = cloud->size();

        // 计算步长 (假设是线性采样)
        int step = (fullSize > 0 && previewSize > 0) ? (fullSize / previewSize) : 1;
        if (step < 1) step = 1;

        // 构建临时 PointXYZRGBNormal 用于显示法线
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previewWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        previewWithNormals->reserve(previewSize);

        for (size_t i = 0; i < previewSize; ++i) {
            const auto& p = previewXYZ->points[i];

            // 回溯原始索引 (近似)
            size_t originalIdx = i * step;
            if (originalIdx >= fullSize) originalIdx = fullSize - 1;

            pcl::PointXYZRGBNormal pn;
            pn.x = p.x; pn.y = p.y; pn.z = p.z;
            pn.rgb = p.rgb;

            // 从全量法线中抽取
            const auto* normals = cloud->getNormalsData();
            if (normals && originalIdx < normals->size()) {
                Eigen::Vector3f n = (*normals)[originalIdx].get();
                pn.normal_x = n.x();
                pn.normal_y = n.y();
                pn.normal_z = n.z();
            }
            previewWithNormals->push_back(pn);
        }

        if (!m_viewer->contains(id))
            m_viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(previewWithNormals, level, scale, id);
        else {
            m_viewer->removePointCloud(id);
            m_viewer->addPointCloudNormals<pcl::PointXYZRGBNormal>(previewWithNormals, level, scale, id);
        }

        // 设置颜色
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                   cloud->normalColor().rf(), cloud->normalColor().gf(),
                                                   cloud->normalColor().bf(), id);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCorrespondences(const Cloud::Ptr &source_points, const Cloud::Ptr &target_points,
                                       const pcl::CorrespondencesPtr &correspondences, const QString &id)
    {
        auto srcPCL = source_points->getPreviewCloud();
        auto tgtPCL = target_points->getPreviewCloud();

        std::string std_id = id.toStdString();

        if (!m_viewer->contains(std_id))
            m_viewer->addCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);
        else
            m_viewer->updateCorrespondences<pcl::PointXYZRGB>(srcPCL, tgtPCL, *correspondences, std_id);

        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, const ct::RGB &rgb)
    {
        std::string std_id = id.toStdString();

        auto pclCloud = cloud->toPCL_XYZRGB();

        if (!m_viewer->contains(std_id))
            m_viewer->addPolygon<PointXYZRGB>(pclCloud, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        else
        {
            m_viewer->removeShape(std_id);
            m_viewer->addPolygon<PointXYZRGB>(pclCloud, rgb.rf(), rgb.gf(), rgb.bf(), std_id);
        }
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                              std_id);

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
    int CloudView::singlePick(const ct::PointXY &pos, const QString& target_cloud_id)
    {
        vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
        vtkRenderer* ren = m_viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer();

        if (!target_cloud_id.isEmpty()){
            std::string id = target_cloud_id.toStdString();
            auto cloud_actor_map = m_viewer->getCloudActorMap();

            if (cloud_actor_map->find(id) != cloud_actor_map->end()){
                vtkProp* actor = cloud_actor_map->at(id).actor;

                picker->InitializePickList();
                picker->AddPickList(actor);
                picker->PickFromListOn();
            }
        }

        picker->Pick(pos.x, pos.y, 0.0, ren);

        int previewIdx = static_cast<int>(picker->GetPointId());
        picker->PickFromListOff();
        if (previewIdx < 0) return -1;

        // 映射回原始索引
        if (!target_cloud_id.isEmpty()) {
            for(const auto& c : m_visible_clouds) {
                if(c->id() == target_cloud_id) {
                    size_t fullSize = c->size();
                    size_t previewSize = c->getPreviewCloud()->size();

                    if (previewSize > 0 && fullSize > previewSize) {
                        size_t step = fullSize / previewSize;
                        if (step < 1) step = 1;

                        // 简单的线性映射
                        size_t realIdx = previewIdx * step;
                        if (realIdx >= fullSize) realIdx = fullSize - 1;

                        return static_cast<int>(realIdx);
                    }
                    // 如果没有 LOD (previewSize == fullSize)，索引是一样的
                    return previewIdx;
                }
            }
        }
        return previewIdx;
    }

    std::vector<int> CloudView::areaPick(const std::vector<PointXY> &poly_points, const Cloud::Ptr &cloud, bool in_out)
    {
        // 获取传入点集合的大小
        int size = poly_points.size();
        if (size < 3) return {};

        // 预计算多边形常数 (Point in Polygon 算法)
        std::vector<float> constant(size);
        std::vector<float> multiple(size);
        int i, j = size - 1;
        for (i = 0; i < size; i++) {
            if (poly_points[j].y == poly_points[i].y) {
                constant[i] = poly_points[i].x;
                multiple[i] = 0;
            } else {
                constant[i] = poly_points[i].x - (poly_points[i].y * poly_points[j].x) / (poly_points[j].y - poly_points[i].y) +
                              (poly_points[i].y * poly_points[i].x) / (poly_points[j].y - poly_points[i].y);
                multiple[i] = (poly_points[j].x - poly_points[i].x) / (poly_points[j].y - poly_points[i].y);
            }
            j = i;
        }

        std::vector<int> indices;
        size_t n_points = cloud->size();

        auto worldToDisplay = [&](const pcl::PointXYZ& pt, double out[3]) {
            m_render->SetWorldPoint(pt.x, pt.y, pt.z, 1.0);
            m_render->WorldToDisplay();
            m_render->GetDisplayPoint(out);
        };

        for (size_t k = 0; k < n_points; k++)
        {
            // 获取全量点
            const auto& pt = cloud->getXYZCloud()->points[k];

            double p[3];
            worldToDisplay(pt, p);

            bool oddNodes = in_out;
            bool current = poly_points[size - 1].y > p[1];
            bool previous;

            for (int m = 0; m < size; m++) {
                previous = current;
                current = poly_points[m].y > p[1];
                if (current != previous)
                    oddNodes ^= (p[1] * multiple[m] + constant[m] < p[0]);
            }
            if (oddNodes) indices.push_back(k);
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
        cloud->setCloudColor(rgb);
        auto renderCloud = cloud->getPreviewCloud();

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(renderCloud);
        m_viewer->updatePointCloud<pcl::PointXYZRGB>(renderCloud, rgb_handler, cloud->id().toStdString());

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
        cloud->setCloudColor(axis);
        auto renderCloud = cloud->getPreviewCloud();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(renderCloud);
        m_viewer->updatePointCloud<pcl::PointXYZRGB>(renderCloud, rgb_handler, cloud->id().toStdString());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetPointCloudColor(const Cloud::Ptr &cloud)
    {
        cloud->restoreColors();

        auto renderCloud = cloud->getPreviewCloud();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(renderCloud);
        m_viewer->updatePointCloud<pcl::PointXYZRGB>(renderCloud, rgb_handler, cloud->id().toStdString());

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
        m_render->GradientBackgroundOff();
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        if (m_auto_render) m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        // 恢复渐变背景
        m_render->GradientBackgroundOn();
        m_render->SetBackground2(0.05, 0.4, 0.6);
        m_render->SetBackground(0.0, 0.05, 0.08);
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

    void CloudView::setPointCloudVisibility(const QString &id, bool visible) {
        std::string std_id = id.toStdString();

        auto cloud_map = m_viewer->getCloudActorMap();
        auto it = cloud_map->find(std_id);
        if (it != cloud_map->end()){
            it->second.actor->SetVisibility(visible ? 1 : 0);
        }

        std::string normal_id = id.toStdString() + "-normals";
        auto it_normal = cloud_map->find(normal_id);
        if (it_normal != cloud_map->end()){
            it_normal->second.actor->SetVisibility(visible ? 1 : 0);
        }

        std::string box_id = id.toStdString() + "-box";
        auto shape_map = m_viewer->getShapeActorMap();
        auto it_box = shape_map->find(box_id);
        if (it_box != shape_map->end()){
            it_box->second->SetVisibility(visible ? 1 : 0);
        }

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
            vtkNew<PCLDisableInteractorStyle> style;
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

    void CloudView::zoomToBounds(const Eigen::Vector3f& min_pt, const Eigen::Vector3f& max_pt){
        double bounds[6] = {
                (double)min_pt.x(), (double)max_pt.x(),
                (double)min_pt.y(), (double)max_pt.y(),
                (double)min_pt.z(), (double)max_pt.z()
        };
        m_render->ResetCamera(bounds);
        m_render->ResetCameraClippingRange();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::mousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            emit mouseLeftPressed(PointXY(this->interactor()->GetEventPosition()[0],
                                          this->interactor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightPressed(PointXY(this->interactor()->GetEventPosition()[0],
                                           this->interactor()->GetEventPosition()[1]));
        }
//        return QVTKOpenGLNativeWidget::mousePressEvent(event);
    }

    void CloudView::mouseReleaseEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            // getViewerPose() 是 PCLVisualizer 类中的一个成员函数。它返回当前视图器的视角或位置的描述，例如相机的位置和方向。
            emit viewerPose(m_viewer->getViewerPose());
            emit mouseLeftReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                           this->interactor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightReleased(PointXY(this->interactor()->GetEventPosition()[0],
                                            this->interactor()->GetEventPosition()[1]));
        }
//        return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    }

    void CloudView::mouseMoveEvent(QMouseEvent *event)
    {
        emit mouseMoved(PointXY(this->interactor()->GetEventPosition()[0],
                                this->interactor()->GetEventPosition()[1]));
//        return QVTKOpenGLNativeWidget::mouseMoveEvent(event);
    }
}