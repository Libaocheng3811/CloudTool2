//
// Created by LBC on 2025/12/23.
//

#ifndef CLOUDTOOL2_FIELDMAPPINGDIALOG_H
#define CLOUDTOOL2_FIELDMAPPINGDIALOG_H

#include "core/common.h"

#include <QDialog>
#include <QTableWidget>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QHeaderView>

namespace ct {
    class FieldMappingDialog : public QDialog {
    Q_OBJECT
    public:
        explicit FieldMappingDialog(const QList<ct::FieldInfo> &fields, QWidget *parent = nullptr)
                : QDialog(parent) {
            setWindowTitle("Field Mapping");
            resize(500, 400);

            QVBoxLayout *layout = new QVBoxLayout(this);
            layout->addWidget(new QLabel("Map file fields to Cloud properties:"));

            m_table = new QTableWidget(fields.size(), 3);
            m_table->setHorizontalHeaderLabels({"File Field", "Type", "Map To"});
            m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

            for (int i = 0; i < fields.size(); ++i) {
                m_table->setItem(i, 0, new QTableWidgetItem(fields[i].name));
                m_table->setItem(i, 1, new QTableWidgetItem(fields[i].type));

                QComboBox *combo = new QComboBox();
                combo->addItem("Ignore");

                // 智能预选
                QString lowerName = fields[i].name.toLower();
                if (lowerName == "x" || lowerName == "y" || lowerName == "z") {
                    combo->addItem("Axis " + fields[i].name.toUpper());
                    combo->setCurrentIndex(1);
                    combo->setCurrentText("Axis " + fields[i].name.toUpper());
                    combo->setEnabled(false); // 坐标强制映射
                } else if (lowerName == "rgba" || lowerName == "rgb") {
                    combo->addItem("Color(Packed)");
                    combo->setCurrentText("Color(Packed)");
                } else if (lowerName.contains("red") || lowerName == "r") {
                    combo->addItem("Red");
                    combo->setCurrentText("Red");
                } else if (lowerName.contains("green") || lowerName == "g") {
                    combo->addItem("Green");
                    combo->setCurrentText("Green");
                } else if (lowerName.contains("blue") || lowerName == "b") {
                    combo->addItem("Blue");
                    combo->setCurrentText("Blue");
                } else if (lowerName == "normal_x" || lowerName == "nx") {
                    combo->addItem("Normal X");
                    combo->setCurrentText("Normal X");
                } else if (lowerName == "normal_y" || lowerName == "ny") {
                    combo->addItem("Normal Y");
                    combo->setCurrentText("Normal Y");
                } else if (lowerName == "normal_z" || lowerName == "nz") {
                    combo->addItem("Normal Z");
                    combo->setCurrentText("Normal Z");
                } else if (lowerName == "curvature") {
                    combo->addItem("Curvature");
                    combo->setCurrentText("Curvature");
                } else {
                    // 默认其他字段作为标量场导入
                    combo->addItem("Scalar Field");
                    if (lowerName == "intensity") {
                        combo->addItem("Intensity");
                        combo->setCurrentText("Intensity");
                    } else {
                        combo->setCurrentText("Scalar Field");
                    }
                }
                m_table->setCellWidget(i, 2, combo);
            }

            layout->addWidget(m_table);

            QPushButton *btnOk = new QPushButton("Import");
            connect(btnOk, &QPushButton::clicked, this, &QDialog::accept);
            layout->addWidget(btnOk);
        }

        MappingResult getMapping() {
            MappingResult res;
            for (int i = 0; i < m_table->rowCount(); ++i) {
                QString name = m_table->item(i, 0)->text();
                QComboBox *cb = qobject_cast<QComboBox *>(m_table->cellWidget(i, 2));
                res.field_map[name] = cb->currentText();
            }
            return res;
        }

    private:
        QTableWidget *m_table;
    };
} // namespace ct
#endif //CLOUDTOOL2_FIELDMAPPINGDIALOG_H
