//
// Created by LBC on 2025/1/9.
//

#ifndef CLOUDTOOL2_KEYPOINTS_H
#define CLOUDTOOL2_KEYPOINTS_H

#include <QDockWidget>


QT_BEGIN_NAMESPACE
namespace Ui { class KeyPoints; }
QT_END_NAMESPACE

class KeyPoints : public QDockWidget {
Q_OBJECT

public:
    explicit KeyPoints(QWidget *parent = nullptr);

    ~KeyPoints() override;

private:
    Ui::KeyPoints *ui;
};


#endif //CLOUDTOOL2_KEYPOINTS_H
