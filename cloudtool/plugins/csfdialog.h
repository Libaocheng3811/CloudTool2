//
// Created by LBC on 2026/1/4.
//

#ifndef CLOUDTOOL2_CSFDIALOG_H
#define CLOUDTOOL2_CSFDIALOG_H

#include <QDialog>
#include <QTabWidget>
#include <QRadioButton>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QTextBrowser>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>

// 参数结构体
struct CSFParams {
  bool bsloopSmooth = false;
  double cloth_resolution = 0.5; // 分辨率
  double class_threshold = 0.5; // 分类阈值
  int rigidness = 3; // 刚性
  int iterations = 500; // 迭代次数
  float time_step = 0.65; // 时间步长
};

class CSFDialog : public  QDialog{
    Q_OBJECT
public:
    CSFDialog(QWidget* parent = nullptr);
    ~CSFDialog() override = default;

    // 获取参数
    CSFParams getParams() const;

private:
    void setupUI();

private:
    // ui控件
    QTextBrowser* m_instruction;
    QTabWidget* m_tabWidget;

    // 基本设置
    QRadioButton* m_rbSteepSlope;
    QRadioButton* m_rbRelief;
    QRadioButton* m_rbFlat;
    QCheckBox* m_chkSlopeProcessing;

    //高级设置
    QDoubleSpinBox* m_dsResolution;
    QDoubleSpinBox* m_dsThreshold;
    QSpinBox* m_spIterations;
};


#endif //CLOUDTOOL2_CSFDIALOG_H
