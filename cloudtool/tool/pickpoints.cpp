//
// Created by LBC on 2024/12/17.
//

// You may need to build the project (run Qt uic code generator) to get "ui_PickPoints.h" resolved

#include "pickpoints.h"
#include "ui_PickPoints.h"

#define PICK_TYPE_TWOPOINTS     0
#define PICK_TYPE_MULTPOINTS    1

#define POLYGONAL_ID        "polygonal"
#define ARROW_ID            "arrow"
#define PICKING_PRE_FLAG    "-picking"
#define PICKING_ADD_FLAG    "picking-"

PickPoints::PickPoints(QWidget *parent) :
        CustomDialog(parent), ui(new Ui::PickPoints),
        is_picking(false),
        pick_start(false),
        m_pick_cloud(new ct::Cloud),
        m_pick_point(-1, -1)
{
    ui->setupUi(this);
    connect(ui->btn_start, &QPushButton::clicked, this, &PickPoints::start);
    connect(ui->btn_add, &QPushButton::clicked, this, &PickPoints::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &PickPoints::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &PickPoints::close);
    ui->cbox_type->setCurrentIndex(0);
}

PickPoints::~PickPoints() {
    delete ui;
}

// 不太明白这个函数是什么时候调用的
void PickPoints::init()
{
    // 将下拉框选项变化的信号与信息更新关联，以便在选项变化时能及时更新信息
    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &PickPoints::updateInfo);
    this->updateInfo(ui->cbox_type->currentIndex());
}

void PickPoints::start()
{
    if (!is_picking)
    {
        std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
        if (selected_clouds.empty())
        {
            printW("Please select a cloud!");
            return;
        }
        is_picking = true;
        // 如果同时选中了多个点云，只对第一个选中的点云进行操作
        m_selected_cloud = selected_clouds.front();
        m_cloudview->removeShape(m_selected_cloud->boxId());
        // 遍历文件树中所有点云，如果不是第一个选中的点云就设置当前点云的勾选状态为未勾选，设置选中状态为未选中
        for (auto& c : m_cloudtree->getAllClouds()) {
            if (c->id() != m_selected_cloud->id()) {
                m_cloudtree->setCloudSelected(c, false);
                m_cloudtree->setCloudChecked(c, false);
            }
        }
        m_cloudview->removeShape(POLYGONAL_ID);

        ui->btn_start->setIcon(QIcon(":/res/icon/stop.svg"));
        connect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &PickPoints::mouseLeftPressed);
        connect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &PickPoints::mouseLeftReleased);
        connect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &PickPoints::mouseRightReleased);
        connect(m_cloudview, &ct::CloudView::mouseMoved, this, &PickPoints::mouseMoved);
        this->updateInfo(ui->cbox_type->currentIndex());
    }
    else
    {
        is_picking = false;
        disconnect(m_cloudview, &ct::CloudView::mouseLeftPressed, this, &PickPoints::mouseLeftPressed);
        disconnect(m_cloudview, &ct::CloudView::mouseLeftReleased, this, &PickPoints::mouseLeftReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseRightReleased, this, &PickPoints::mouseRightReleased);
        disconnect(m_cloudview, &ct::CloudView::mouseMoved, this, &PickPoints::mouseMoved);
        ui->btn_start->setIcon(QIcon(":/res/icon/start.svg"));
        m_cloudview->removeShape(ARROW_ID);
        m_cloudview->removeShape(POLYGONAL_ID);
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_cloudview->clearInfo();
        this->updateInfo(ui->cbox_type->currentIndex());
    }
}

void PickPoints::add()
{
    if (m_selected_cloud == nullptr)
    {
        printW("Please select a cloud!");
        return;
    }
    if (m_pick_cloud->empty())
    {
        printW("Please pick point first!");
        return;
    }
    // 创建一个新的智能指针new_cloud，指向与m_pick_cloud（也就是选择的点）相同的对象
    ct::Cloud::Ptr new_cloud = m_pick_cloud->makeShared();
    // 在视图中移除当前的点云，以避免视觉上重复显示点云
    m_cloudview->removePointCloud(new_cloud->id());
    // 设置new_cloud的新ID
    new_cloud->setId(PICKING_ADD_FLAG + m_selected_cloud->id());
    // 将新点云追加到点云树中，便于管理使用
    m_cloudtree->appendCloud(m_selected_cloud, new_cloud, true);
    this->reset();
    printI(QString("Add picking cloud[id:%1] done.").arg(new_cloud->id()));
}

// 重置当前PickPoints类中所有相关状态和视图
void PickPoints::reset()
{
    if (is_picking) this->start();
    pick_start = false;
    // 移除形状，也就是视图中绘制的形状，例如箭头和矩形框
    m_cloudview->removeShape(POLYGONAL_ID);
    m_cloudview->removeShape(ARROW_ID);
    m_pick_cloud->clear();
    m_selected_cloud = nullptr;
}

void PickPoints::updateInfo(int index)
{
    if (index == PICK_TYPE_TWOPOINTS)
    {
        if (is_picking)
            m_cloudview->showInfo("PickPoints [ON] (two points picking)", 1);
        else
            m_cloudview->showInfo("PickPoints [OFF] (two points picking)", 1);
        m_cloudview->showInfo("Left click : pick point", 2);
    }
    else if (index == PICK_TYPE_MULTPOINTS)
    {
        if (is_picking)
            m_cloudview->showInfo("PickPoints [ON] (multil points picking)", 1);
        else
            m_cloudview->showInfo("PickPoints [OFF] (multil points picking)", 1);
        m_cloudview->showInfo("Left click : add points / Right click : close", 2);
    }
}

void PickPoints::mouseLeftPressed(const ct::PointXY &pt)
{
    if (!pick_start)
    {
        // 从视图器中拾取点，返回的start是该点的索引
        int start = m_cloudview->singlePick(pt);
        if (start == 1 || start == -1) return;
        m_pick_point = pt;
        // 清空m_pick_cloud中的点，确保每次选择都从空的点云开始
        m_pick_cloud->clear();
        // 从已选的点云中获取索引为start对应的点
        ct::PointXYZRGBN start_pt = m_selected_cloud->points[start];
        // 将点加入被选中的点的集合中
        m_pick_cloud->push_back(start_pt);
        m_pick_cloud->setId(m_selected_cloud->id() + PICKING_PRE_FLAG);
        // 移除箭头形状，清除先前选择的状态
        m_cloudview->removeShape(ARROW_ID);
        // 将选择的点添加到视图中，更新显示点的状态
        m_cloudview->addPointCloud(m_pick_cloud);
        m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
        m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->pointSize() + 3);
        pick_start = true;

        // 显示点的索引，以及点的坐标
        if (ui->cbox_type->currentIndex() == PICK_TYPE_TWOPOINTS)
            m_cloudview->showInfo(tr("Start Point[%1]: 2%, 3%, 4%").arg(start).arg(start_pt.x, 0, 'g', 3).
                    arg(start_pt.y, 0, 'g', 3).arg(start_pt.z, 0, 'g', 3), 3, ct::Color::Yellow);
        else
            m_cloudview->showInfo(tr("Pick Point[%1]: %2, %3, %4").arg(start).arg(start_pt.x, 0, 'g', 3).
                    arg(start_pt.y, 0, 'g', 3).arg(start_pt.z, 0, 'g', 3), 3, ct::Color::Yellow);
    }
}

void PickPoints::mouseLeftReleased(const ct::PointXY &pt)
{
    if (pick_start)
    {
        // 如果鼠标右键释放时的位置和之前已经选择的点的位置一样，则直接返回
        if ((m_pick_point.x == pt.x) && (m_pick_point.y == pt.y))
        {
            return;
        }
        else
        {
            // 获取鼠标左键释放时位置处点的索引，singlePick(pt)是拾取位置pt的点，并返回索引
            int end = m_cloudview->singlePick(pt);
            if (end == 1 || end == -1) return;
            // 从已选点云中获取对应索引的点
            ct::PointXYZRGBN end_pt = m_selected_cloud->points[end];
            m_pick_cloud->push_back(end_pt);
            // 清除视图中之前的选点矩形框
            m_cloudview->removeShape(POLYGONAL_ID);
            m_cloudview->addPointCloud(m_pick_cloud);
            m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
            m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->size() + 3);
            if (ui->cbox_type->currentIndex() == PICK_TYPE_TWOPOINTS)
            {
                ct::PointXYZRGBN start_pt = m_pick_cloud->points.front();
                // 绘制箭头
                m_cloudview->addArrow(end_pt, start_pt, ARROW_ID, true, ct::Color::Green);
                float x = start_pt.x - end_pt.x;
                float y = start_pt.y - end_pt.y;
                float z = start_pt.z - end_pt.z;
                float distance = sqrt( x * x + y * y + z * z);
                m_cloudview->showInfo(tr("End Point[%1]: %2, %3, %4").arg(end).arg(end_pt.x, 0, 'g', 3).
                        arg(end_pt.y, 0, 'g', 3).arg(end_pt.z, 0, 'g', 3), 4, ct::Color::Yellow);
                m_cloudview->showInfo(tr("Distance: 1%").arg(distance, 0, 'g', 3), 5, ct::Color::Yellow);
                pick_start = false;
            }
        }
    }
}

void PickPoints::mouseRightReleased(const ct::PointXY&)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == PICK_TYPE_MULTPOINTS)
        {
            m_cloudview->removeShape(POLYGONAL_ID);
            m_cloudview->addPointCloud(m_pick_cloud);
            m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
            m_cloudview->setPointCloudSize(m_pick_cloud->id(), m_pick_cloud->pointSize() + 3);
            m_cloudview->addPolygon(m_pick_cloud, POLYGONAL_ID, ct::Color::Green);
            pick_start = false;
        }
    }
}

void PickPoints::mouseMoved(const ct::PointXY &pt)
{
    if (pick_start)
    {
        int tmp = m_cloudview->singlePick(pt);
        if (tmp == 1 || tmp == -1) return;
        // 创建临时点云，指向m_pick_cloud
        ct::Cloud::Ptr tmp_cloud = m_pick_cloud->makeShared();
        // 根据索引将对应的点添加到临时点云中
        tmp_cloud->push_back(m_selected_cloud->points[tmp]);
        // 绘制多边形
        m_cloudview->addPolygon(tmp_cloud, POLYGONAL_ID, ct::Color::Green);
    }
}