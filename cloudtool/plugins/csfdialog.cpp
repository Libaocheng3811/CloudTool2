//
// Created by LBC on 2026/1/4.
//

#include "csfdialog.h"

#include <QPushButton>

CSFDialog::CSFDialog(QWidget *parent) :  QDialog(parent){
    setWindowTitle("Cloth Simulation Filter");
    resize(500, 600);
    setupUI();
}

void CSFDialog::setupUI() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // 顶部说明
    m_instruction = new QTextBrowser();
    m_instruction->setHtml(
            "<h3>CSF Plugin Instruction</h3>"
            "<p><b>Cloth Simulation Filter (CSF)</b> is a tool to extract ground points...</p>"
            "<p>Reference: Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. "
            "<i>Remote Sensing</i>. 2016; 8(6):501.</p>"
            );
    m_instruction->setFixedHeight(150);
    mainLayout->addWidget(m_instruction);

    // 参数设置
    m_tabWidget = new QTabWidget();

    // Tab1 general settings
    QWidget* tabGeneral = new QWidget();
    QVBoxLayout* layGeneral = new QVBoxLayout(tabGeneral);

    QGroupBox* grpScenes = new QGroupBox("Scenes");
    QVBoxLayout* layScenes = new QVBoxLayout(grpScenes);

    m_rbSteepSlope = new QRadioButton("Steep Slope");
    m_rbRelief = new QRadioButton("Relief");
    m_rbFlat = new QRadioButton("Flat");
    m_rbRelief->setChecked(true);

    m_rbSteepSlope->setIcon(QIcon(":/res/images/CSF_hun.png"));
    m_rbRelief->setIcon(QIcon(":/res/images/CSF_gun.png"));
    m_rbFlat->setIcon(QIcon(":/res/images/CSF_ft.png"));

    layScenes->addWidget(m_rbSteepSlope);
    layScenes->addWidget(m_rbRelief);
    layScenes->addWidget(m_rbFlat);

    layGeneral->addWidget(grpScenes);

    m_chkSlopeProcessing = new QCheckBox("Slope Processing");
    m_chkSlopeProcessing->setIcon(QIcon(":/res/images/CSF_pp.png"));
    layGeneral->addWidget(m_chkSlopeProcessing);
    layGeneral->addStretch();

    m_tabWidget->addTab(tabGeneral, "General Settings");

    // Tab2 advanced settings
    QWidget* tabAdvanced = new QWidget();
    QGridLayout* layAdvanced = new QGridLayout(tabAdvanced);

    // resolution
    layAdvanced->addWidget(new QLabel("Cloth Resolution"), 0, 0);
    m_dsResolution = new QDoubleSpinBox();
    m_dsResolution->setRange(0.1, 100.0);
    m_dsResolution->setValue(1.0);
    m_dsResolution->setSuffix("m"); // 单位：m
    layAdvanced->addWidget(m_dsResolution, 0, 1);
    QLabel* lblResInfo = new QLabel("Refers to thr grid size of cloth...");
    lblResInfo->setWordWrap(true);
    lblResInfo->setStyleSheet("color : gray; font-size : 10px; ");
    layAdvanced->addWidget(lblResInfo, 1, 0, 1, 2);

    // max iterations
    layAdvanced->addWidget(new QLabel("Max Iterations"), 2, 0);
    m_spIterations = new QSpinBox();
    m_spIterations->setRange(1, 5000);
    m_spIterations->setValue(500);
    layAdvanced->addWidget(m_spIterations, 2, 1);

    // class threshold
    layAdvanced->addWidget(new QLabel("Class Threshold"), 3, 0);
    m_dsThreshold = new QDoubleSpinBox();
    m_dsThreshold->setRange(0.01, 10.0);
    m_dsThreshold->setValue(0.5);
    m_dsThreshold->setSuffix("m");
    layAdvanced->addWidget(m_dsThreshold, 3, 1);
    QLabel* lblThresholdInfo = new QLabel("Refers to the threshold of the distance between the point and the cloth...");
    lblThresholdInfo->setWordWrap(true);
    lblThresholdInfo->setStyleSheet("color : gray; font-size : 10px;");
    layAdvanced->addWidget(lblThresholdInfo, 4, 0, 1, 2);

    // 底部跳虫
    layAdvanced->setRowStretch(5, 1);
    m_tabWidget->addTab(tabAdvanced, "Advanced Settings");

    mainLayout->addWidget(m_tabWidget);

    // 底部按钮
    QWidget* btnWidget = new QWidget();
    QHBoxLayout* btnLayout = new QHBoxLayout(btnWidget);
    QPushButton* btnOK = new QPushButton("OK");
    QPushButton* btnCancel = new QPushButton("Cancel");

    connect(btnOK, &QPushButton::clicked, this, &QDialog::accept);
    connect(btnCancel, &QPushButton::clicked, this, &QDialog::reject);

    btnLayout->addWidget(btnOK);
    btnLayout->addWidget(btnCancel);
    btnLayout->addStretch();

    mainLayout->addWidget(btnWidget);
}

CSFParams CSFDialog::getParams() const {
    CSFParams params;
    params.bsloopSmooth = m_chkSlopeProcessing->isChecked();
    params.cloth_resolution = m_dsResolution->value();
    params.class_threshold = m_dsThreshold->value();
    params.iterations = m_spIterations->value();

    // Rigidness 映射 (1=Flat, 2=Relief, 3=SteepSlope)
    if (m_rbFlat->isChecked()) params.rigidness = 3;
    else if (m_rbRelief->isChecked()) params.rigidness = 2;
    else params.rigidness = 1; // steep slope

    return params;

}
