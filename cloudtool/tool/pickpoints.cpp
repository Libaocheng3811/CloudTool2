//
// Created by LBC on 2024/12/17.
//

// You may need to build the project (run Qt uic code generator) to get "ui_PickPoints.h" resolved

#include "pickpoints.h"
#include "ui_PickPoints.h"

#include <QTimer>

//#define PICK_TYPE_TWOPOINTS     0
//#define PICK_TYPE_MULTPOINTS    1

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

    this->layout()->setSizeConstraint(QLayout::SetFixedSize);
    ui->infoBrowser->setVisible(false);
    ui->infoBrowser->setStyleSheet("QTextBrowser { border: 1px solid #CCCCCC; background-color: #F0F0F0; border-radius: 4px; }");

    connect(ui->btn_start, &QPushButton::clicked, this, &PickPoints::start);
    connect(ui->btn_add, &QPushButton::clicked, this, &PickPoints::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &PickPoints::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &PickPoints::close);

    ui->cbox_type->setCurrentIndex(0);

    QTimer::singleShot(0, this, [this](){
        this->adjustSize();
    });
}

PickPoints::~PickPoints() {
    delete ui;
}

void PickPoints::init()
{
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

        if(ui->infoBrowser->isVisible()){
            ui->infoBrowser->setVisible(false);
            this->adjustSize();
        }

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
        printW("Please pick point first，no points picked to save!");
        return;
    }

    ct::Cloud::Ptr new_cloud = m_pick_cloud->makeShared();
    new_cloud->setPointSize(10);
    new_cloud->setCloudColor(ct::Color::Red);

    // 在视图中移除当前的点云，以避免视觉上重复显示点云
//    m_cloudview->removePointCloud(new_cloud->id());
    QString unique_suffix = QString::number(QDateTime::currentMSecsSinceEpoch() % 10000);
    QString new_id = QString("pciked-%1-%2").arg(m_selected_cloud->id()).arg(unique_suffix);

    new_cloud->setId(new_id);
    new_cloud->setInfo(m_selected_cloud->info());

    // 将新点云追加到点云树中，便于管理使用
    m_cloudtree->appendCloud(m_selected_cloud, new_cloud, true);

    m_cloudview->removePointCloud(m_pick_cloud->id());
    m_pick_cloud->clear();
    m_cloudview->removeShape(ARROW_ID);
    m_cloudview->removeShape(POLYGONAL_ID);

    if(ui->infoBrowser->isVisible()){
        ui->infoBrowser->setVisible(false);
        this->adjustSize();
    }

    printI(QString("Add picking cloud[id:%1] done.").arg(new_cloud->id()));
}

// 重置当前PickPoints类中所有相关状态和视图
void PickPoints::reset()
{
    if (is_picking) this->start();
    pick_start = false;
    if (!m_pick_cloud->empty()){
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_pick_cloud->clear();
    }
    m_selected_cloud = nullptr;

    // 移除形状，也就是视图中绘制的形状，例如箭头和矩形框
    m_cloudview->removeShape(POLYGONAL_ID);
    m_cloudview->removeShape(ARROW_ID);

    if(ui->infoBrowser->isVisible()){
        ui->infoBrowser->setVisible(false);
        this->adjustSize();
    }
}

void PickPoints::updateInfo(int index)
{
    QString modeStr = "";
    if (index == PICK_SINGLE) modeStr = "Single Point";
    else if (index == PICK_PAIR) modeStr = "Two Points(Diatance)";
    else modeStr = "Polygon";

    QString status = is_picking ? "[ON]" : "[OFF]";
    m_cloudview->showInfo(QString("PickPoints %1(%2)").arg(status).arg(modeStr), 1);

    //切换模式时隐藏面板
    if (index != PICK_SINGLE){
        if (ui->infoBrowser->isVisible()){
            ui->infoBrowser->setVisible(false);
            this->adjustSize();
        }
    }
}

void PickPoints::updatePanelInfo(int index, const ct::PointXYZRGBN &pt, const ct::Cloud::Ptr& cloud) {
    // 基础样式
    QString html = "<body style='font-family: Microsoft YaHei, Arial, sans-serif; font-size: 9pt; color: #333;'>";

    // 标题和索引
    html += QString("<div style='background-color: #E0E0E0; padding: 4px; border-radius: 3px;'><b>Index:</b> <span style='color:#0055AA'>%1</span></div>").arg(index);
    html += "<table width='100%' border='0' cellspacing='0' cellpadding='2' style='margin-top: 5px;'>";

    // --- 动态生成表头 ---
    html += "<tr>";
    html += "<td style='color:#888; font-size:8pt;'>COORD</td>"; // XYZ 永远存在

    bool showRGB = cloud->hasRGB();
    if (showRGB) {
        html += "<td style='color:#888; font-size:8pt;'>RGB</td>";
    }

    bool showNormal = cloud->hasNormals();
    if (showNormal) {
        html += "<td style='color:#888; font-size:8pt;'>NORMAL</td>";
    }
    html += "</tr>";

    // --- 第一行 (X, R, Nx) ---
    html += QString("<tr><td>X: %1</td>").arg(pt.x, 0, 'f', 3);
    if (showRGB)    html += QString("<td>R: %1</td>").arg((int)pt.r);
    if (showNormal) html += QString("<td>Nx: %1</td>").arg(pt.normal_x, 0, 'f', 2);
    html += "</tr>";

    // --- 第二行 (Y, G, Ny) ---
    html += QString("<tr><td>Y: %1</td>").arg(pt.y, 0, 'f', 3);
    if (showRGB)    html += QString("<td>G: %1</td>").arg((int)pt.g);
    if (showNormal) html += QString("<td>Ny: %1</td>").arg(pt.normal_y, 0, 'f', 2);
    html += "</tr>";

    // --- 第三行 (Z, B, Nz) ---
    html += QString("<tr><td>Z: %1</td>").arg(pt.z, 0, 'f', 3);
    if (showRGB)    html += QString("<td>B: %1</td>").arg((int)pt.b);
    if (showNormal) html += QString("<td>Nz: %1</td>").arg(pt.normal_z, 0, 'f', 2);
    html += "</tr>";

    html += "</table>";

    QStringList scalarNames = cloud->getScalarFieldNames();

    if (!scalarNames.isEmpty()) {
        html += "<div style='margin-top: 8px; border-top: 1px solid #DDD; padding-top: 4px;'></div>";
        html += "<table width='100%' border='0' cellspacing='0' cellpadding='2'>";

        for (const QString& name : scalarNames) {
            const std::vector<float>* data = cloud->getScalarField(name);
            if (data && index < data->size()) {
                float val = (*data)[index];

                // 简单的两列布局：字段名 | 数值
                html += QString("<tr><td style='color:#666;'>%1:</td><td><b>%2</b></td></tr>")
                        .arg(name)
                        .arg(val, 0, 'f', 6);
            }
        }
        html += "</table>";
    }

    html += "</body>";
    ui->infoBrowser->setHtml(html);

    if (!ui->infoBrowser->isVisible()) {
        ui->infoBrowser->setVisible(true);
        this->adjustSize();
    }
}

void PickPoints::mouseLeftPressed(const ct::PointXY &pt)
{
    int index = m_cloudview->singlePick(pt, m_selected_cloud->id());
    if (index < 0) return;

    ct::PointXYZRGBN  current_pt = m_selected_cloud->points[index];
    int mode = ui->cbox_type->currentIndex();

    if (mode == PICK_SINGLE){
        m_pick_point = pt;
        m_pick_cloud->clear();
        m_pick_cloud->push_back(current_pt);
        m_pick_cloud->setId(m_selected_cloud->id() + PICKING_PRE_FLAG);

        //高亮显示红点
        m_pick_cloud->setPointSize(15);
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_cloudview->addPointCloud(m_pick_cloud);
        m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);

        updatePanelInfo(index, current_pt, m_selected_cloud);

        printI(QString("Picked Point: %1").arg(index));
    }
    else if (mode == PICK_PAIR){
        if(ui->infoBrowser->isVisible()){
            ui->infoBrowser->setVisible(false);
            this->adjustSize();
        }

        if (!pick_start){
            //第一点
            m_pick_point = pt;
            m_pick_cloud->clear();
            m_pick_cloud->push_back(current_pt);

            m_pick_cloud->setPointSize(15);
            m_cloudview->removePointCloud(m_pick_cloud->id());
            m_cloudview->addPointCloud(m_pick_cloud);
            m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
            m_cloudview->removeShape(ARROW_ID);

            pick_start = true;
            m_cloudview->showInfo("Pick End Point...", 3);
        } // 第二点在 mouseLeftReleased 中处理
    }
    else if(mode == PICK_MULTI){
        if(ui->infoBrowser->isVisible()){
            ui->infoBrowser->setVisible(false);
            this->adjustSize();
        }

        m_pick_point = pt;
        if (!pick_start) {
            m_pick_cloud->clear();
            pick_start = true;
        }
        m_pick_cloud->push_back(current_pt);

        // 更新显示
        m_pick_cloud->setPointSize(15);
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_cloudview->addPointCloud(m_pick_cloud);
        m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);
        m_cloudview->removeShape(ARROW_ID);
    }
}

void PickPoints::mouseLeftReleased(const ct::PointXY &pt)
{
    // 仅处理两点模式的第二点
    if (ui->cbox_type->currentIndex() == PICK_PAIR && pick_start){
        if(m_pick_point.x == pt.x && m_pick_point.y == pt.y) return;

        int index = m_cloudview->singlePick(pt, m_selected_cloud->id());
        if (index < 0) return;

        ct::PointXYZRGBN end_pt = m_selected_cloud->points[index];
        m_pick_cloud->push_back(end_pt);

        //画线
        m_pick_cloud->setPointSize(15);
        m_cloudview->removePointCloud(m_pick_cloud->id());
        m_cloudview->addPointCloud(m_pick_cloud);
        m_cloudview->setPointCloudColor(m_pick_cloud, ct::Color::Red);

        ct::PointXYZRGBN start_pt = m_pick_cloud->points.front();
        m_cloudview->addArrow(end_pt, start_pt, ARROW_ID, true, ct::Color::Green);

        float distance = (start_pt.getVector3fMap() - end_pt.getVector3fMap()).norm();

        m_cloudview->showInfo(QString("Distance: %1").arg(distance), 5, ct::Color::Yellow);

        pick_start = false;

        if(ui->infoBrowser->isVisible()){
            ui->infoBrowser->setVisible(false);
            this->adjustSize();
        }
    }
}

void PickPoints::mouseRightReleased(const ct::PointXY&)
{
    if (pick_start)
    {
        if (ui->cbox_type->currentIndex() == PICK_MULTI)
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
    if (!pick_start) return;

    int tmp = m_cloudview->singlePick(pt, m_selected_cloud->id());
    if (tmp < 0){
        if (ui->cbox_type->currentIndex() == PICK_PAIR) m_cloudview->removeShape(ARROW_ID);
        if (ui->cbox_type->currentIndex() == PICK_MULTI) m_cloudview->removeShape(POLYGONAL_ID);
        return;
    }

    ct::PointXYZRGBN  current_hover_pt = m_selected_cloud->points[tmp];
    int mode = ui->cbox_type->currentIndex();

    if (mode == PICK_PAIR){
        if (m_pick_cloud->empty()) return;

        ct::PointXYZRGBN start_pt = m_pick_cloud->points[0];
        m_cloudview->addArrow(current_hover_pt, start_pt, ARROW_ID, true, ct::Color::Green);
    }
    else if (mode == PICK_MULTI){
        ct::Cloud::Ptr tmp_cloud = m_pick_cloud->makeShared();
        tmp_cloud->push_back(current_hover_pt);

        m_cloudview->addPolygon(tmp_cloud, POLYGONAL_ID, ct::Color::Green);
    }
}