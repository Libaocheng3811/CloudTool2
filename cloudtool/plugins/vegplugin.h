//
// Created by LBC on 2026/1/6.
//

#ifndef CLOUDTOOL2_VEGPLUGIN_H
#define CLOUDTOOL2_VEGPLUGIN_H

#include <QDialog>
#include <QThread>

#include "base/customdialog.h"
#include "modules/vegfilter.h"

QT_BEGIN_NAMESPACE
namespace Ui { class VegPlugin; }
QT_END_NAMESPACE

class VegPlugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit VegPlugin(QWidget *parent = nullptr);

    ~VegPlugin() override;

    void init() override;

private slots:
    void onIndexChanged(int index);
    void onApply();
    void onFilterDone(const ct::Cloud::Ptr& veg_cloud, const ct::Cloud::Ptr& non_veg_cloud, float time);

signals:
    void requestVegFilter(int type, double threshold);

private:
    Ui::VegPlugin *ui;
    QThread m_thread;
    ct::VegetationFilter* m_filter;
    ct::Cloud::Ptr m_cloud;
};

#endif //CLOUDTOOL2_VEGPLUGIN_H
