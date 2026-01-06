#include "base/cloudtree.h"
#include "base/fieldmappingdialog.h"
#include "base/txtimportdialog.h"
#include "base/txtexportdialog.h"

#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMenu>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPushButton>
#include <QHeaderView>
#include <QSpinBox>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>
#include <QComboBox>
#include <QApplication>

namespace ct
{
    // ROOT_PATH指向程序根目录路径
    CloudTree::CloudTree(QWidget *parent)
        : CustomTree(parent),
        m_path(ROOT_PATH),
        m_thread(this),
        m_tree_menu(nullptr)
    {
        // register meat type
        // qRegisterMetaType 函数用于注册一个自定义类型，以便它可以被用于 Qt 的元对象系统
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");
        qRegisterMetaType<QList<ct::FieldInfo>>("QList<ct::FieldInfo>");
        qRegisterMetaType<QMap<QString, QString>>("QMap<QString, QString>&");
        qRegisterMetaType<ct::TxtImportParams>("ct::TxtImportParams");
        qRegisterMetaType<ct::TxtExportParams>("ct::TxtExportParams");

        // move to thread
        m_fileio = new FileIO;
        // 将m_fileio对象移动到m_thread线程中
        m_fileio->moveToThread(&m_thread);
        connect(&m_thread, &QThread::finished, m_fileio, &QObject::deleteLater);
        connect(m_fileio, &FileIO::loadCloudResult, this, &CloudTree::loadCloudResult);
        connect(m_fileio, &FileIO::saveCloudResult, this, &CloudTree::saveCloudResult);
        connect(this, &CloudTree::loadPointCloud, m_fileio, &FileIO::loadPointCloud);
        connect(this, &CloudTree::savePointCloud, m_fileio, &FileIO::savePointCloud);
        connect(this, &CloudTree::itemClicked, this, &CloudTree::itemClickedEvent);
        connect(this, &CloudTree::itemSelectionChanged, this, &CloudTree::itemSelectionChangedEvent);
        connect(m_fileio, &FileIO::requestFieldMapping, this, &CloudTree::onFieldMappingRequested, Qt::BlockingQueuedConnection);
        connect(m_fileio, &FileIO::requestTxtImportSetup, this, &CloudTree::onTxtImportRequested, Qt::BlockingQueuedConnection);
        connect(m_fileio, &FileIO::requestTxtExportSetup, this, &CloudTree::onTxtExportRequested, Qt::BlockingQueuedConnection);

        // 设置窗口部件接受拖放操作
        this->setAcceptDrops(true);

        m_thread.start();
    }

    CloudTree::~CloudTree()
    {
        m_thread.quit();
        if (!m_thread.wait(3000))
        {
            m_thread.terminate();
            m_thread.wait();
        }
    }

    void CloudTree::addCloud()
    {
        /**
         * @note 文件过滤器
         * 文件过滤器的写法遵循一定的格式，filter = "filterName(pattern1 pattern2 ...)";
         * filterName 是一个描述性的名字，用于在文件对话框中显示给用户。如 all, ply
         * pattern1 pattern2 ... 是一个或多个文件模式，它们定义了过滤器匹配的文件类型, 如 *.*, *.ply
         */
        // 定义了一个文件过滤器
        QString filter = "All Supported(*.ply *.pcd *.las *.laz *.obj *.ifs *.txt *.asc *.xyz);;All Files(*.*)";
        // 打开文件对话框,可以选择多个文件
        QStringList filePathList = QFileDialog::getOpenFileNames(this, tr("open cloud files"), m_path, filter);
        if (filePathList.isEmpty()) return;

        showProgress("Loading Point Cloud...");

        bindWorker(m_fileio);

        for (auto& i : filePathList)
            emit loadPointCloud(i);
    }

    void CloudTree::insertCloud(const ct::Index &index, const Cloud::Ptr &cloud, bool selected)
    {
        // check cloud id
        if (cloud == nullptr) return;
        if (m_cloudview->contains(cloud->id()))
        {
            int k = QMessageBox::warning(this, "WARNING", "Rename the exists id?", QMessageBox::Yes, QMessageBox::Cancel);
            if (k == QMessageBox::Yes)
            {
                bool ok = false;
                QString res = QInputDialog::getText(this, "Rename", "", QLineEdit::Normal, cloud->id(), &ok);
                if (ok)
                {
                    if (res == cloud->id() || m_cloudview->contains(res))
                    {
                        printE(QString("The cloud id[%1] already exists!").arg(res));
                        return;
                    }
                    cloud->setId(res);
                }
                else
                {
                    printW("Add cloud canceled.");
                    return;
                }
            }
            else
            {
                printW("Add cloud canceled.");
                return;
            }
        }

        // update cloud_vec
        /**
         * @brief 结构树索引说明
         * 1、当index.row 小于或等于 -1 或大于顶级项的数量时，意味着没有有效的行索引。m_cloud_vec 的末尾添加一个新的内部向量，并把点云 cloud 添加到这个新的向量中
         *    这通常意味着在树的根级别创建一个新的节点。
         * 2、当 index.row 有效时，会检查 index.col 是否有效。如果 index.col 在有效范围内，会在 m_cloud_vec[index.row] 的指定列位置插入点云 cloud
         *    这通常意味着在树的某个特定节点下插入一个新的子节点。
         * 3、如果 index.col 不在有效范围内，代码会将点云 cloud 添加到 m_cloud_vec[index.row] 的末尾。
         *    这通常意味着在树的某个节点下追加一个新的子节点。
         */
        // 行索引无效，在根级别插入新节点
        if (index.row <= -1 || index.row > topLevelItemCount())
        {
            std::vector<Cloud::Ptr> temp;
            temp.push_back(cloud);
            m_cloud_vec.push_back(temp);
        }
        // 行索引有效，检查列索引
        else
        {
            // 列索引有效，在row节点下的第col列插入cloud， topLevelItem返回row行的顶层节点，childCount()返回该节点下的子节点数量
            if ((index.col > -1) && (index.col < topLevelItem(index.row)->childCount()))
                m_cloud_vec[index.row].insert(m_cloud_vec[index.row].begin() + index.col, cloud);
            // 列索引无效，在row节点下的最后插入cloud
            else
                m_cloud_vec[index.row].push_back(cloud);
        }
        // add treewidget item
        addItem(index, cloud->info().absolutePath(), cloud->id(), selected);
        m_cloudview->addPointCloud(cloud);
        m_cloudview->resetCamera();
        printI(QString("Add cloud[id:%1] done.").arg(cloud->id()));
    }

    void CloudTree::updateCloud(const Cloud::Ptr &cloud, const Cloud::Ptr &new_cloud, bool update_name)
    {
        if (cloud == nullptr || new_cloud == nullptr)
            return;
        // 如果cloud和new_cloud不是同一个对象，交换他们的内容
        if (cloud != new_cloud)
            cloud->swap(*new_cloud);
        // 更新点云
        cloud->update();
        // 根据ID获取点云在文件树中的索引
        Index i = index(cloud->id());
        if (update_name)
            // 将点云名称更新为new_cloud的ID
            renameCloud(i, new_cloud->id());
        m_cloudview->addPointCloud(cloud);
        // 如果item(i)返回为nullptr, 再访问它的成员函数isSelected()，不会报错吗？访问了空指针的成员函数
        if (item(i)->isSelected())
            m_cloudview->addBox(cloud);
        printI(QString("Update cloud[id:%1, size:%2] to new cloud[id:%3, size:%4] done.")
                .arg(cloud->id()).arg(cloud->size()).arg(new_cloud->id()).arg(new_cloud->size()));
    }

    void CloudTree::removeCloud(const ct::Index &index)
    {
        Cloud::Ptr cloud = getCloud(index);
        emit removedCloudId(cloud->id());
        // 移除点云，法线和包围盒
        m_cloudview->removePointCloud(cloud->id());
        m_cloudview->removeShape(cloud->id());
        m_cloudview->removePointCloud(cloud->normalId());
        // 移除项
        removeItem(index);
        // 如果索引行只有一个点云对象，则移除整行
        if (m_cloud_vec[index.row].size() == 1)
            m_cloud_vec.erase(m_cloud_vec.begin() + index.row);
        // 否则，移除指定列的点云对象，
        else
            m_cloud_vec[index.row].erase(m_cloud_vec[index.row].begin() + index.col);
        printI(QString("Remove cloud[id:%1] done.").arg(cloud->id()));

    }

    void CloudTree::removeAllClouds()
    {
        for (auto i : m_cloud_vec)
            for (auto j : i)
                emit removedCloudId(j->id());
        m_cloudview->removeAllPointClouds();
        m_cloudview->removeAllShapes();
        this->clear();
        this->m_cloud_vec.clear();
        std::vector<std::vector<Cloud::Ptr>>().swap(m_cloud_vec);
        printI("remove all clouds done.");
    }

    void CloudTree::saveCloud(const ct::Index &index)
    {
        Cloud::Ptr cloud = getCloud(index);
        QString filter = "PLY(*.ply);;PCD(*.pcd);;LAS(*.las);;TXT(*.txt)";
        QString filepath = QFileDialog::getSaveFileName(this, tr("Save cloud file"), cloud->id(), filter);
        if (filepath.isEmpty()) return;

        QMessageBox message_box(QMessageBox::NoIcon, "Saved format", tr("Save in binary or ascii format?"),
                                QMessageBox::NoButton, this);
        message_box.addButton(tr("Ascii"), QMessageBox::ActionRole);
        message_box.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
        message_box.addButton(QMessageBox::Cancel);
        int k = message_box.exec();
        if (k == QMessageBox::Cancel)
        {
            printW("Save cloud canceled.");
            return;
        }

        showProgress("Saving Point Cloud...");
        bindWorker(m_fileio);

        emit savePointCloud(cloud, filepath, k);
    }

    void CloudTree::mergeSelectedClouds()
    {
        std::vector<Cloud::Ptr> clouds = getSelectedClouds();
        // 如果选择的点云数量小于2，打印警告信息
        if (clouds.size() <= 1)
        {
            printW("The number of clouds to merge is not enough!");
            return;
        }
        // 新创建点云对象merge_cloud，循环通过重载运算符将点云合并到merge_cloud中
        Cloud::Ptr merge_cloud(new Cloud);
        for (auto& i : clouds)
            *merge_cloud += *i;
        merge_cloud->setId(MERGE_ADD_FLAG + clouds.front()->id());
        merge_cloud->setInfo(clouds.front()->info());
        merge_cloud->update();
        // 将合并后的点云添加到视图中
        appendCloud(merge_cloud);
        printI(QString("Merge clouds to new cloud[id:%1] done.").arg(merge_cloud->id()));
    }

    void CloudTree::setCloudChecked(const Cloud::Ptr &cloud, bool checked)
    {
        if (cloud == nullptr)
            return;
        Index i = index(cloud->id());
        // 如果当前状态与要设置的状态一致，则直接返回，不做任何操作
        if (checked && (item(i)->checkState(0) == Qt::Checked))
            return;
        if ((!checked) && (item(i)->checkState(0) == Qt::Unchecked))
            return;
        // 否则，根据传入的目标状态设置选中状态
        this->setItemChecked(i, checked);
        // 如果是选中，则添加点云和包围盒，否则移除点云、包围盒、法线
        if (checked)
        {
            m_cloudview->addPointCloud(cloud);
            m_cloudview->addBox(cloud);
        }
        else
        {
            m_cloudview->removePointCloud(cloud->id());
            m_cloudview->removeShape(cloud->boxId());
            m_cloudview->removePointCloud(cloud->normalId());
        }
    }

    void CloudTree::setCloudSelected(const Cloud::Ptr &cloud, bool selected)
    {
        if (cloud == nullptr)
            return;
        Index i = index(cloud->id());
        item(i)->setSelected(selected);
    }

    void CloudTree::cloneCloud(const Index& index)
    {
        Cloud::Ptr clone_cloud = getCloud(index)->makeShared();
        clone_cloud->setId(CLONE_ADD_FLAG + clone_cloud->id());
        clone_cloud->setInfo(clone_cloud->info());
        // 在克隆时不需要调用update方法，因为创建一个已有点云的精确副本，而这个副本在创建时已经包含了所有原始点云的状态，包括任何必要的信息和数据。
        appendCloud(clone_cloud);
        printI(QString("Clone cloud [id:%1] done.").arg(clone_cloud->id()));
    }

    void CloudTree::renameCloud(const ct::Index &index, const QString &name)
    {
        // 获取当前索引点云对象
        Cloud::Ptr cloud = getCloud(index);
        // 检查是否已经存在新的点云ID
        if (m_cloudview->contains(name))
        {
            printW(QString("The Cloud already id[%1] exists!").arg(name));
            return;
        }
        // 设置文件树中新名称
        item(index)->setText(0, name);

        m_cloudview->removePointCloud(cloud->id());
        m_cloudview->removePointCloud(cloud->normalId());
        m_cloudview->removeShape(cloud->boxId());
        // 更新点云ID
        cloud->setId(name);
        printI(QString("Rename cloud [id:%1] to new name[id:%2] done. ").arg(cloud->id()).arg(name));
    }

    void CloudTree::loadCloudResult(bool success, const Cloud::Ptr &cloud, float time)
    {
        if (!success)
            printE("load the file failed!");
        else
        {
            printI(QString("Load the file [path:%1] done, take time %2 ms.").arg(cloud->info().absoluteFilePath()).arg(time));
            m_path = cloud->info().path();
            appendCloud(cloud);
        }

        closeProgress();
    }

    void CloudTree::saveCloudResult(bool success, const QString &path, float time)
    {
        if (!success)
            printE("Save the file failed!");
        else
        {
            m_path = path;
            printI(QString("Save the file [path:%1] done, take time %2 ms.").arg(path).arg(time));
        }

        closeProgress();
    }

    void CloudTree::itemClickedEvent(QTreeWidgetItem *item, int)
    {
        // 获取点击节点的索引
        std::vector<Index> index = getClickedIndexes(item);
        for (auto &i : index)
        {
            Cloud::Ptr cloud = getCloud(i);
            // 如果节点复选框是选中状态，添加对应的点云数据和包围盒
            if (item->checkState(0) == Qt::Checked)
            {
                m_cloudview->addPointCloud(cloud);
                m_cloudview->addBox(cloud);
            }
            // 如果节点复选框是未选中状态，移除对应的点云数据、包围盒、法线数据。
            else
            {
                m_cloudview->removePointCloud(cloud->id());
                m_cloudview->removeShape(cloud->boxId());
                m_cloudview->removePointCloud(cloud->normalId());
            }
        }
    }

    void CloudTree::itemSelectionChangedEvent()
    {
        std::vector<Index> all = getAllIndexes();
        std::vector<Index> indexes = getSelectedIndexes();
        for (auto& i : all)
        {
            std::vector<Index>::const_iterator it = std::find(indexes.begin(), indexes.end(), i);
            Cloud::Ptr cloud = getCloud(i);
            if (it != indexes.end())
                m_cloudview->addBox(cloud);
            else
                m_cloudview->removeShape(cloud->boxId());
        }

        // update table
        if (m_table == nullptr)
            return;

        QString id, type, size_str, resolution_str;

        const int COLOR_MODE_ROW = 7;

        m_table->removeCellWidget(4, 1);
        m_table->removeCellWidget(5, 1);
        m_table->removeCellWidget(6, 1);
        m_table->removeCellWidget(COLOR_MODE_ROW, 1);
        if (indexes.size() == 0)
        {
            id = type = size_str = resolution_str = "";
            // 在渲染窗口中显示点云id
            m_cloudview->showCloudId("");
        }
        else
        {
            // 有选中点云的情况下，属性表显示第一个索引的点云信息
            Cloud::Ptr update_cloud = getCloud(indexes.front());
            id = update_cloud->id();
            type = update_cloud->type();
            resolution_str = QString::number(update_cloud->resolution());
            size_str = QString::number(update_cloud->size());
            m_cloudview->showCloudId(update_cloud->id());

            // point_size
            // 添加一个QSpinBox控件用于设置点云点的大小
            QSpinBox *point_size = new QSpinBox;
            point_size->setRange(1, 99);
            point_size->setValue(update_cloud->pointSize());
            connect(point_size, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int value)
                    {
                        // 更新点的大小
                        update_cloud->setPointSize(value);
                        // 更新点云视图大小
                        m_cloudview->setPointCloudSize(update_cloud->id(), value);
                    });
            m_table->setCellWidget(4, 1, point_size);

            // opacity
            QDoubleSpinBox* opacity = new QDoubleSpinBox;
            opacity->setSingleStep(0.1);
            opacity->setRange(0, 1);
            opacity->setValue(update_cloud->opacity());
            connect(opacity, static_cast<void (QDoubleSpinBox::*)(double )>(&QDoubleSpinBox::valueChanged), [=](double value)
                    {
                        update_cloud->setOpacity(value);
                        m_cloudview->setPointCloudOpacity(update_cloud->id(), value);
                    });
            m_table->setCellWidget(5, 1, opacity);

            // has normals
            QCheckBox* show_normals = new QCheckBox;
            QDoubleSpinBox* scale = new QDoubleSpinBox;
            scale->setSingleStep(0.01);
            scale->setRange(0, 9999);
            scale->setValue(0.01);

            // 如果点云有法线，设置法线复选框状态为可选
            if (update_cloud->hasNormals())
                show_normals->setEnabled(true);
            else
                show_normals->setEnabled(false);

            connect(scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value )
                    {
                        // 当前点云有法线并且复选框被选中时，添加点云法线
                        if (update_cloud->hasNormals() && show_normals->isChecked())
                            m_cloudview->addPointCloudNormals(update_cloud, 1, value);
                    });
            connect(show_normals, &QCheckBox::stateChanged, [=](int state)
                    {
                        // 如果复选框状态为选中，添加点云法线； 否则，删除点云法线
                        if (state) m_cloudview->addPointCloudNormals(update_cloud, 1, scale->value());
                        else m_cloudview->removeShape(update_cloud->normalId());
                    });

            QHBoxLayout* layout = new QHBoxLayout;
            layout->addWidget(show_normals);
            layout->addWidget(scale);
            // 使用 addStretch() 方法可以在布局中插入一段可伸缩的空间，这段空间可以根据窗口大小的变化而调整。
            layout->addStretch();
            QWidget* normals_widget = new QWidget;
            normals_widget->setLayout(layout);
            normals_widget->layout()->setMargin(0);
            m_table->setCellWidget(6, 1, normals_widget);

            if (m_table->rowCount() <= COLOR_MODE_ROW) m_table->setRowCount(COLOR_MODE_ROW + 1);

            QComboBox* color_mode = new QComboBox;
            color_mode->addItem("RGB (Default)"); // 恢复原始 RGB 或默认黑色
            color_mode->addItem("x");
            color_mode->addItem("y");
            color_mode->addItem("z");

            // 动态添加自定义字段 (如 Intensity)
            QStringList fields = update_cloud->getScalarFieldNames();
            for (const QString& f : fields) {
                color_mode->addItem(f);
            }

            connect(color_mode, &QComboBox::currentTextChanged, [=](const QString& text)
            {
                if (text == "RGB (Default)") {
                    // 调用 CloudView 的重置函数，它会内部调用 cloud->restoreColors()
                    m_cloudview->resetPointCloudColor(update_cloud);
                }
                else if (text == "x" || text == "y" || text == "z") {
                    // cloud->setCloudColor(axis) 内部会先 backupColors()
                    update_cloud->setCloudColor(text.toLower());
                    m_cloudview->addPointCloud(update_cloud); // 刷新显示
                }
                else {
                    // 自定义字段着色（Cloud::updateColorByField 内部会先 backupColors()）
                    update_cloud->updateColorByField(text);
                    m_cloudview->addPointCloud(update_cloud); // 刷新显示
                }
            });

//            m_table->setItem(COLOR_MODE_ROW, 0, new QTableWidgetItem("Color Mode"));
            m_table->setCellWidget(COLOR_MODE_ROW, 1, color_mode);

        }
        m_table->setItem(0, 1, new QTableWidgetItem(id));
        m_table->setItem(1, 1, new QTableWidgetItem(type));
        m_table->setItem(2, 1, new QTableWidgetItem(size_str));
        m_table->setItem(3, 1, new QTableWidgetItem(resolution_str));
    }

    void CloudTree::showProgress(const QString &message) {
        if (!m_processing_dialog){
            // 寻找最顶层的窗口作为父窗口，确保模态对话框居中显示
            QWidget* topLevel = this->window();
            m_processing_dialog = new ProcessingDialog(topLevel);
            m_processing_dialog->setWindowModality(Qt::WindowModal);
        }

        m_processing_dialog->reset();
        m_processing_dialog->setMessage(message);
        m_processing_dialog->show();
        QApplication::processEvents(); // 强制刷新UI
    }

    void CloudTree::closeProgress() {
        if (m_processing_dialog){
            m_processing_dialog->close();
            delete m_processing_dialog;
            m_processing_dialog = nullptr;
        }
    }

    void CloudTree::bindWorker(QObject *worker) {
        if (!m_processing_dialog || !worker) return;

        // Worker -> Dialog (进度更新)
        bool ok = connect(worker, SIGNAL(progress(int)), m_processing_dialog, SLOT(setProgress(int)), Qt::UniqueConnection);
        std::cout << "connect progress: " << ok << std::endl;

        // Dialog -> Worker (取消请求),信号和槽连接方式为直接连接，确保能够快速响应取消请求
        ok = connect(m_processing_dialog, SIGNAL(cancelRequested()), worker, SLOT(cancel()), Qt::DirectConnection);
        std::cout << "connect cancelRequested: " << ok << std::endl;

        connect(m_processing_dialog, &ProcessingDialog::cancelRequested, this, &CloudTree::closeProgress);
    }

    void CloudTree::onFieldMappingRequested(const QList<ct::FieldInfo>& fields, QMap<QString, QString>& result)
    {
        // 这个函数运行在主线程 (UI线程)
        FieldMappingDialog dlg(fields, this);
        if (dlg.exec() == QDialog::Accepted) {
            // 用户点击 OK，获取结果
            FieldMappingDialog::MappingResult res = dlg.getMapping();
            result = res.field_map;
        } else {
            // 用户取消，返回空结果
            result.clear();
        }
    }

    void CloudTree::onTxtImportRequested(const QStringList& preview_lines, ct::TxtImportParams& params){
        TxtImportDialog dlg(preview_lines, this);
        if (dlg.exec() == QDialog::Accepted) {
            params = dlg.getParams();
        } else{
            params.col_map.clear();
        }
    }

    void CloudTree::onTxtExportRequested(const QStringList &available_fields, ct::TxtExportParams &params) {
        TxtExportDialog dlg(available_fields, this);
        if (dlg.exec() == QDialog::Accepted){
            params = dlg.getParams();
        }
        else{
            params.selected_fields.clear(); //标志取消
        }
    }
}

