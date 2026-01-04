//
// Created by LBC on 2026/1/4.
//

// You may need to build the project (run Qt uic code generator) to get "ui_csf.h" resolved

#include "csf.h"
#include "ui_csf.h"


csf::csf(QWidget *parent) :
        QDialog(parent), ui(new Ui::csf) {
    ui->setupUi(this);
}

csf::~csf() {
    delete ui;
}
