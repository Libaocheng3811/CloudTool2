//
// Created by LBC on 2025/1/9.
//

// You may need to build the project (run Qt uic code generator) to get "ui_KeyPoints.h" resolved

#include "keypoints.h"
#include "ui_KeyPoints.h"


KeyPoints::KeyPoints(QWidget *parent) :
        QDockWidget(parent), ui(new Ui::KeyPoints) {
    ui->setupUi(this);
}

KeyPoints::~KeyPoints() {
    delete ui;
}
