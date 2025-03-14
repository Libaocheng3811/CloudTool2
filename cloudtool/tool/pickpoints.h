//
// Created by LBC on 2024/12/17.
//

#ifndef CLOUDTOOL2_PICKPOINTS_H
#define CLOUDTOOL2_PICKPOINTS_H

#include "base/customdialog.h"
#include "base/common.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class PickPoints;
}
QT_END_NAMESPACE

class PickPoints : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit PickPoints(QWidget *parent = nullptr);

    ~PickPoints() override;

    virtual void init();
    // 开始选点模式
    void start();
    // 将选中的点取出，单独成一项
    void add();
    // 重置功能
    virtual void reset();
    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    void updateInfo(int index);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

private:
    Ui::PickPoints *ui;
    bool is_picking;
    bool pick_start;
    // 选中的点云
    ct::Cloud::Ptr m_selected_cloud;
    // 存储已经选择的点
    ct::Cloud::Ptr m_pick_cloud;
    // 拾取的点
    ct::PointXY m_pick_point;
};


#endif //CLOUDTOOL2_PICKPOINTS_H
