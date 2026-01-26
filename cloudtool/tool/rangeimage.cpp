//
// Created by LBC on 2025/1/9.
//

#include "rangeimage.h"
#include "ui_rangeimage.h"

#include <pcl/visualization/common/float_image_utils.h>
#include <vtkCamera.h>

#define RANGE_IMAGE_FLAG    "range_image"

RangeImage::RangeImage(QWidget *parent) :
    CustomDialog(parent), ui(new Ui::RangeImage),
    m_range_image(new ct::RangeImage),
    m_image_data(vtkSmartPointer<vtkImageData>::New()),
    m_ren(vtkSmartPointer<vtkRenderer>::New()),
    m_actor(vtkSmartPointer<vtkImageActor>::New()),
    m_cloud(new ct::Cloud)
{
    // setupUi(this) 是 Qt 中用于将 UI 组件设置到当前窗口或控件中的函数。this 指的是当前窗口类的实例，通常是继承自 QMainWindow 或 QWidget 的类。
    // 就是将你设计的UI界面加载到你当前的类上，关联起来
    ui->setupUi(this);
    // 调整当前渲染器中活动摄像机的视角
    m_ren->GetActiveCamera()->Azimuth(180.0);
    // 将渲染器m_ren添加到渲染窗口中
    ui->qvtkwidget->GetRenderWindow()->AddRenderer(m_ren);
    // 设置背景色为白色
    m_ren->SetBackground(1, 1, 1);
    connect(ui->checkBox, &QCheckBox::clicked, [=](bool checked)
    {
        // 如果勾选了复选框，在视图器中显示由范围图像生成的点云
        if (checked && !m_cloud->empty())
        {
            m_cloudview->addPointCloudFromRangeImage(m_range_image, RANGE_IMAGE_FLAG, ct::Color::Green);
            m_cloudview->setPointCloudSize(RANGE_IMAGE_FLAG, m_cloud->pointSize() + 2);
        }
        else
        {
            // 如果没有勾选，移除视图器中显示的由范围图像生成的点云
            m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
        }
    });
}

RangeImage::~RangeImage() {
    delete ui;
}

void RangeImage::init()
{
    // 当视图发生变化时，更新深度图像
    connect(m_cloudview, &ct::CloudView::viewerPose, this, &RangeImage::updateRangeImage);
}

void RangeImage::updateRangeImage(Eigen::Affine3f viewer_pose)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        m_image_data = vtkSmartPointer<vtkImageData>::New();
        // 从渲染器中移除图像
        m_ren->RemoveActor(m_actor);
        // 重新渲染窗口
        ui->qvtkwidget->GetRenderWindow()->Render();
        m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
        return;
    }
    m_cloud = selected_clouds.front();
    /**
     * @brief 从点云数据生成范围图像（深度图像）
     */
    // createFromPointCloud是pcl::RangeImage 类的一个成员函数，用于从点云创建范围图像。
    // 需要将Cloud转换为PCL点云
    m_range_image->createFromPointCloud(*m_cloud->toPCL_XYZRGBN(), pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), viewer_pose);
    // 返回一个指向范围图像中所有距离值的数组的指针。每个距离值表示从相机到对应像素点的距离。
    float* ranges_array = m_range_image->getRangesArray();
    // getVisualImage是 pcl::visualization::FloatImageUtils 类的一个静态方法，用于将距离值数组转换为可视化的RGB图像。
    unsigned char* rgbImage = pcl::visualization::FloatImageUtils::getVisualImage(ranges_array, m_range_image->width, m_range_image->height);
    // 将 rgbImage 从 unsigned char* 类型转换为 void* 类型。这是为了与VTK的 SetVoidArray 方法兼容，该方法接受一个 void* 类型的参数。
    void* data = const_cast<void*> (reinterpret_cast<const void*> (rgbImage));
    // 设置图像数据的范围，这里设置为范围图像的宽度和高度
    m_image_data->SetExtent(0, m_range_image->width - 1, 0, m_range_image->height - 1, 0, 0);
    // 分配图像数据的标量数组
    m_image_data->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    // 设置图像数据的标量数组
    m_image_data->GetPointData()->GetScalars()->SetVoidArray(data, 3 * m_range_image->width * m_range_image->height, 1);

    m_actor->SetInputData(m_image_data);
    m_actor->Update();
    m_ren->AddActor(m_actor);
    m_ren->ResetCamera();
    // this->resize(2 * m_range_image->width + 6, 2 * m_range_image->height + 31);
    ui->qvtkwidget->GetRenderWindow()->Render();
    if (ui->checkBox->isChecked())
    {
        m_cloudview->addPointCloudFromRangeImage(m_range_image, RANGE_IMAGE_FLAG, ct::Color::Green);
        m_cloudview->setPointCloudSize(RANGE_IMAGE_FLAG, m_cloud->pointSize() + 2);
    }
}

void RangeImage::reset()
{
    m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
    disconnect(m_cloudview, &ct::CloudView::viewerPose, this, &RangeImage::updateRangeImage);
}
