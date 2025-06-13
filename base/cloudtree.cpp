#include "base/cloudtree.h"

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

namespace ct
{
    // ROOT_PATH指向程序根目录路径
    CloudTree::CloudTree(QWidget *parent)
        : CustomTree(parent),
        m_path(ROOT_PATH),
        m_thread(this),
        m_tree_menu(nullptr)
    {
        QHeaderView* header = this->header();
        header->setStyleSheet(R"(
            QHeaderView::section {
                background-color: #dddddd;
                color: black;
                padding: 5px;
                border-radius: 3px;
                border: 1px solid #cccccc;
                font-weight: bold;
                text-align: center;
            }
        )");
        // register meat type
        // qRegisterMetaType 函数用于注册一个自定义类型，以便它可以被用于 Qt 的元对象系统
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr &");
        qRegisterMetaType<Cloud::Ptr>("Cloud::Ptr");

        // move to thread
        m_fileio = new FileIO;
        // 将m_fileio对象移动到m_thread线程中
        m_fileio->moveToThread(&m_thread);
        connect(&m_thread, &QThread::finished, m_fileio, &QObject::deleteLater);
        connect(m_fileio, &FileIO::loadCloudResult, this, &CloudTree::loadCloudResult);
        connect(m_fileio, &FileIO::saveCloudResult, this, &CloudTree::saveCloudResult);
        connect(this, &CloudTree::loadPointCloud, m_fileio, &FileIO::loadPointCloud);
        connect(this, &CloudTree::savePointCloud, m_fileio, &FileIO::savePointCloud);
        m_thread.start();

        connect(this, &CloudTree::itemClicked, this, &CloudTree::itemClickedEvent);
        connect(this, &CloudTree::itemSelectionChanged, this, &CloudTree::itemSelectionChangedEvent);

        // 设置窗口部件接受拖放操作
        this->setAcceptDrops(true);
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
        QString filter = "all(*.*);;ply(*.ply);;pcd(*.pcd)";
        // 打开文件对话框,可以选择多个文件
        QStringList filePathList = QFileDialog::getOpenFileNames(this, tr("open cloud files"), m_path, filter);
        if (filePathList.isEmpty())
        {
            return;
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->show();

        // 这种循环方式特别适合于遍历容器中的所有元素，如数组、向量、列表、字符串等
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
        // 发射信号由谁接受呢？
        emit removedCloudId(cloud->id());
        // 移除点云，法线和包围盒
        m_cloudview->removePointCloud(cloud->id());
        m_cloudview->removeShape(cloud->id());
        m_cloudview->removePointCloud(cloud->normalId());
        // 移除项
        removeItem(index);
        // 如果索引行只有一个点云对象，则移除整行
        if (m_cloud_vec[index.row].size() == 1)
            // erase() 函数通过提供迭代器的方式，从 m_cloud_vec 中删除对应的行
            // 这里不可以写成m_cloud_vec.erase(index.row);错误
            // 因为erase 函数需要一个迭代器，指向要被删除的元素。
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
        // clear()清空 CloudTree 对象的所有树节点。这通常意味着清除了所有的树节点显示。
        this->clear();
        // 清空存储点云的向量，释放内存
        this->m_cloud_vec.clear();
        // 创建一个临时的空向量，通过交换的方式高效地清空 m_cloud_vec 的内容，并确保它占用的内存被正确释放。
        // swap 是 std::vector 提供的成员函数，它的作用是交换两个向量的内容。
        std::vector<std::vector<Cloud::Ptr>>().swap(m_cloud_vec);
        printI("remove all clouds done.");
    }

    void CloudTree::saveCloud(const ct::Index &index)
    {
        Cloud::Ptr cloud = getCloud(index);
        QString filter = "ply(*.ply);;pcd(*.pcd)";
        // QFileDialog::getSaveFileName 显示一个文件保存对话框，允许用户选择保存位置和文件名。
        // 对话框的标题为 “Save cloud file”，默认文件名为当前点云的 ID，使用前面定义的 filter 进行文件类型过滤。
        QString filepath = QFileDialog::getSaveFileName(this, tr("Save cloud file"), cloud->id(), filter);
        if (filepath.isEmpty())
            return;
        // 显示格式选择对话框
        // QMessageBox::NoIcon：指定消息框没有图标。"Saved format"：消息框的标题。
        // tr("Save in binary or ascii format?")：消息框的内容. QMessageBox::NoButton：没有默认的按钮，意味着没有一个按钮会自动被选择。
        // this：指向当前的窗口对象，表示这个消息框是该窗口的子窗口。
        QMessageBox message_box(QMessageBox::NoIcon, "Saved format", tr("Save in binary or ascii format?"),
                                QMessageBox::NoButton, this);
        // 添加按钮，向消息框中添加一个 “Ascii” 按钮。按钮类型为 QMessageBox::ActionRole
        message_box.addButton(tr("Ascii"), QMessageBox::ActionRole);
        // 添加一个 “Binary” 按钮，并将其设置为默认按钮
        message_box.addButton(tr("Binary"), QMessageBox::ActionRole)->setDefault(true);
        message_box.addButton(QMessageBox::Cancel);
        // 模态窗口，用户做出选择后，返回的值将被存储在 k 变量中
        int k = message_box.exec();
        if (k == QMessageBox::Cancel)
        {
            printW("Save cloud canceled.");
            return;
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->show();
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
        // 设置点云id，
        // front()它用于访问向量的第一个元素，并返回对第一个元素的引用
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
        // 移除旧点云ID相关数据，点云、法线、包围盒。
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
            // 这里只给了一个参数，是因为在appendCloud声明中，第二个参数有默认值，所以在调用时可以省略，编译器会自动使用默认值
            appendCloud(cloud);
        }
        if (m_progress_bar != nullptr)
            m_progress_bar->close();
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
        if (m_progress_bar != nullptr)
            m_progress_bar->close();
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
        // update box
        // 获取所有节点的索引和被选中节点的索引
        std::vector<Index> all = getAllIndexes();
        std::vector<Index> indexes = getSelectedIndexes();
        for (auto& i : all)
        {
            // std::vector<Index>::const_iterator是声明一个迭代器，用于遍历std::vector<Index>类型的容器，
            // 还有比如，std::vector<int>::iterator it = vec.begin(); 这声明的就是可修改的迭代器
            // const_iterator表示一个常量迭代器，意味着它只能读取元素，不能修改元素。
            // it是存储std::find结果的迭代器变量。如果没有找到，it将等于indexes.end()。
            std::vector<Index>::const_iterator it = std::find(indexes.begin(), indexes.end(), i);
            Cloud::Ptr cloud = getCloud(i);
            // 如果找到了对应索引，添加对应的点云包围盒；如果没有找到，就移除包围盒
            // ????不理解
            if (it != indexes.end())
                m_cloudview->addBox(cloud);
            else
                m_cloudview->removeShape(cloud->boxId());
        }

        // update table
        if (m_table == nullptr)
            return;
        QString id, type, format, size, resolution;
        // 当前没有选中点云的情况下，也就是选中点云的索引为空
        if (indexes.size() == 0)
        {
            id = type = size = resolution = "";
            // 当你调用removeCellWidget(4, 1)时，它会移除第5行第2列单元格中的小部件,
            // 如果该单元格没有小部件，或者小部件不是通过setCellWidget方法添加的，则此函数不会有任何效果。
            m_table->removeCellWidget(4, 1); // point size
            m_table->removeCellWidget(5, 1); //opacity
            m_table->removeCellWidget(6, 1); //normals
            // 在渲染窗口中显示点云id
            m_cloudview->showCloudId("");
        }
        else
        {
            // 有选中点云的情况下，属性表显示第一个索引的点云信息
            Cloud::Ptr update_cloud = getCloud(indexes.front());
            id = update_cloud->id();
            type = update_cloud->type();
            // QString::number()是Qt中QString类的静态函数，它用于将数值转换为QString类型的字符串。
            // 这个函数可以接受不同类型的数值参数，包括int、float、double等，并将其转换为对应的字符串表示。
            resolution = QString::number(update_cloud->resolution());
            size = QString::number(update_cloud->size());
            m_cloudview->showCloudId(update_cloud->id());

            // point_size
            // 添加一个QSpinBox控件用于设置点云点的大小
            QSpinBox *point_size = new QSpinBox;
            point_size->setRange(1, 99);
            point_size->setValue(update_cloud->pointSize());

            // &QSpinBox::valueChanged是QSpinBox的一个信号，当值改变时发出该信号，
            // static_cast 是 C++ 中的一种类型转换方式，用于将一个类型转换为另一个兼容类型。QSpinBox::* 是一个指向 QSpinBox 类的成员的指针
            // &QSpinBox::valueChanged表示指向 QSpinBox 类的 valueChanged 信号的指针
            // 将 QSpinBox 的 valueChanged 信号（实际上是一个类成员）转换为一个函数指针，该指针指向一个返回类型为 void 并接受一个 int 参数的成员函数。
            // 这个int参数就是要传递给匿名函数的，对应匿名函数中的(int value)
            // 在 Qt 中，可以使用 lambda 表达式作为槽函数，而信号的接收者通常是上下文（如类的成员变量）中隐含的
            connect(point_size, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int value)
                    {
                        // 更新点的大小
                        update_cloud->setPointSize(value);
                        // 更新点云视图大小
                        m_cloudview->setPointCloudSize(update_cloud->id(), value);
                    });

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
            QWidget* normals = new QWidget;
            normals->setLayout(layout);
            normals->layout()->setMargin(0);
            m_table->setCellWidget(4, 1, point_size);
            m_table->setCellWidget(5, 1, opacity);
            m_table->setCellWidget(6, 1, normals);
        }
        // 在表格的特定位置插入一个新的QTableWidgetItem对象
        // 设置单元格时，是从数据区域开始计数的，不计算表头，比如索引(0,1),对应的是数据区域的第一行第二列的单元格
        // QTableWidgetItem 类有一个构造函数，可以接受一个 QString 参数，该参数将被用作单元格中显示的文本。id就作为显示的文本
        m_table->setItem(0, 1, new QTableWidgetItem(id));
        m_table->setItem(1, 1, new QTableWidgetItem(type));
        m_table->setItem(2, 1, new QTableWidgetItem(size));
        m_table->setItem(3, 1, new QTableWidgetItem(resolution));
    }
}

