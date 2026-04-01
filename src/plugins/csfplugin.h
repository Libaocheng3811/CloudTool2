//
// Created by LBC on 2026/1/4.
//

#ifndef CLOUDTOOL2_CSFPLUGIN_H
#define CLOUDTOOL2_CSFPLUGIN_H

#include <QDialog>
#include <QThread>

#include "widgets/customdialog.h"
#include "modules/csffilter.h"


QT_BEGIN_NAMESPACE
namespace Ui { class CSFPlugin; }
QT_END_NAMESPACE

class CSFPlugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit CSFPlugin(QWidget *parent = nullptr);

    ~CSFPlugin() override;

    void init() override;

private slots:
    void onApply();
    void onCancel();

public slots:
    void onFilterDone(const ct::Cloud::Ptr& ground_cloud, const ct::Cloud::Ptr& off_ground_cloud, float time);

signals:
    void requestCSF(bool bSloopSmooth, float time_step, double class_threshold,
                    double cloth_resolution, int rigidness, int iterations);

private:
    Ui::CSFPlugin *ui;
    QThread m_thread;
    ct::CSFFilter* m_filter;
    ct::Cloud::Ptr m_cloud;
};


#endif //CLOUDTOOL2_CSFPLUGIN_H
