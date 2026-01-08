//
// Created by LBC on 2026/1/8.
//

// You may need to build the project (run Qt uic code generator) to get "ui_globalshiftdialog.h" resolved

#include "base/globalshiftdialog.h"

#include <utility>
#include "ui_globalshiftdialog.h"


GlobalShiftDialog::GlobalShiftDialog(Eigen::Vector3d  original_min,
                                     const Eigen::Vector3d& suggested_shift,
                                     QWidget *parent)
                                     : QDialog(parent), ui(new Ui::GlobalShiftDialog), m_original_point(std::move(original_min)) {
    ui->setupUi(this);

    ui->lblOriginalX->setText(QString::number(original_min.x(), 'f', 4));
    ui->lblOriginalY->setText(QString::number(original_min.y(), 'f', 4));
    ui->lblOriginalZ->setText(QString::number(original_min.z(), 'f', 4));

    ui->spinShiftX->setValue(suggested_shift.x());
    ui->spinShiftY->setValue(suggested_shift.y());
    ui->spinShiftZ->setValue(suggested_shift.z());

    connect(ui->spinShiftX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);
    connect(ui->spinShiftY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);
    connect(ui->spinShiftZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &GlobalShiftDialog::onShiftChanged);

    connect(ui->btnOk, &QPushButton::clicked, this, &GlobalShiftDialog::onBtnOKClicked);
    connect(ui->btnCancel, &QPushButton::clicked, this, &GlobalShiftDialog::onBtnCancelClicked);

    onShiftChanged();
}

GlobalShiftDialog::~GlobalShiftDialog() {
    delete ui;
}

Eigen::Vector3d GlobalShiftDialog::getShiftValue() const {
    return Eigen::Vector3d(ui->spinShiftX->value(), ui->spinShiftY->value(), ui->spinShiftZ->value());
}

bool GlobalShiftDialog::isSkipped() const {
    return m_skipped;
}

void GlobalShiftDialog::onShiftChanged() {
    // Local = Original + Shift
    double lx = m_original_point.x() + ui->spinShiftX->value();
    double ly = m_original_point.y() + ui->spinShiftY->value();
    double lz = m_original_point.z() + ui->spinShiftZ->value();

    ui->lblLocalX->setText(QString::number(lx, 'f', 4));
    ui->lblLocalY->setText(QString::number(ly, 'f', 4));
    ui->lblLocalZ->setText(QString::number(lz, 'f', 4));
}
void GlobalShiftDialog::onBtnOKClicked() {
    m_skipped = false;
    accept();
}

void GlobalShiftDialog::onBtnCancelClicked() {
    m_skipped = true;
    this->reject();
}