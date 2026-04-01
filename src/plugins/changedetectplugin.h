//
// Created by LBC on 2026/1/18.
//

#ifndef CLOUDTOOL2_CHANGEDETECTDIALOG_H
#define CLOUDTOOL2_CHANGEDETECTDIALOG_H

#include <QDialog>
#include <QThread>
#include <QMetaType>

#include "core/common.h"
#include "modules/distancecalculator.h"
#include "widgets/customdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class ChangeDetectPlugin; }
QT_END_NAMESPACE

Q_DECLARE_METATYPE(ct::DistanceParams)

class ChangeDetectPlugin : public ct::CustomDialog {
Q_OBJECT

public:
    explicit ChangeDetectPlugin(QWidget *parent = nullptr);

    ~ChangeDetectPlugin() override;

    void init() override;

private slots:
    void onApply();

    void onCancel();

    void onMethodChanged(int index);

    void onFilterDone(const std::vector<float>& distances, float time);

signals:
    void requestChangeDetect(ct::Cloud::Ptr reference, ct::Cloud::Ptr compared, ct::DistanceParams params);

private:
    Ui::ChangeDetectPlugin *ui;

    QThread m_thread;

    ct::DistanceCalculator* m_calculator;
    ct::Cloud::Ptr m_refCloud;
    ct::Cloud::Ptr m_compCloud;

    // 缓存当前参数
    double m_threshold;
};


#endif //CLOUDTOOL2_CHANGEDETECTDIALOG_H
