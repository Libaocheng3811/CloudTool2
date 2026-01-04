//
// Created by LBC on 2026/1/4.
//

#ifndef CLOUDTOOL2_CSF_H
#define CLOUDTOOL2_CSF_H

#include <QDialog>


QT_BEGIN_NAMESPACE
namespace Ui { class csf; }
QT_END_NAMESPACE

class csf : public QDialog {
    Q_OBJECT

public:
    explicit csf(QWidget *parent = nullptr);

    ~csf() override;

private:
    Ui::csf *ui;
};


#endif //CLOUDTOOL2_CSF_H
