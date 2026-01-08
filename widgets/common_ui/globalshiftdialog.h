//
// Created by LBC on 2026/1/8.
//

#ifndef CLOUDTOOL2_GLOBALSHIFTDIALOG_H
#define CLOUDTOOL2_GLOBALSHIFTDIALOG_H

#include <Eigen/Dense>
#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class GlobalShiftDialog; }
QT_END_NAMESPACE

class GlobalShiftDialog : public QDialog {
Q_OBJECT

public:
    /**
    * @brief 构造函数
    * @param original_min 原始点云的最小点坐标 (用于显示)
    * @param suggested_shift 建议的偏移值 (通常为负数，例如 -370000)
    */
    explicit GlobalShiftDialog(Eigen::Vector3d  original_min,
                               const Eigen::Vector3d& suggested_shift,
                               QWidget *parent = nullptr);

    ~GlobalShiftDialog() override;

    Eigen::Vector3d getShiftValue() const;

    // 用户是否选择跳过大坐标偏移，使用大坐标显示
    bool isSkipped() const;

private slots:
    void onShiftChanged();

    void onBtnOKClicked();
    void onBtnCancelClicked();

private:
    Ui::GlobalShiftDialog *ui;
    Eigen::Vector3d m_original_point;
    bool m_skipped = false;
};


#endif //CLOUDTOOL2_GLOBALSHIFTDIALOG_H
