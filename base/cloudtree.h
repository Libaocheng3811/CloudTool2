#ifndef CLOUDTOOL2_CLOUDTREE_H
#define CLOUDTOOL2_CLOUDTREE_H

#include "base/customtree.h"
#include "fileio.h"

#include <QMenu>
#include <QInputDialog>
#include <QThread>

#define CLONE_ADD_FLAG "clone-"
#define MERGE_ADD_FLAG "merge-"

namespace ct
{
    class CT_EXPORT CloudTree : public CustomTree
    {
    Q_OBJECT
    public:
        explicit CloudTree(QWidget* parent = nullptr);

        ~CloudTree() override;

        /**
         * @brief 添加点云
         */
        void addCloud();

        /**
         * @brief 更新点云
         */
        void updateCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool update_name = false);

        /**
         * @brief 追加点云到同一层
         */
        void appendCloud(const Cloud::Ptr& cloud, const Cloud::Ptr& new_cloud, bool selected = false)
        {
            new_cloud->update();
            Index i(index(cloud->id()).row, -1);
            insertCloud(i, new_cloud, selected);
        }

        /**
         * @brief 追加点云到新一层
         */
        void appendCloud(const Cloud::Ptr& new_cloud, bool selected = false)
        {
            insertCloud(Index(-1,-1), new_cloud, selected);
        }

        /**
         * @brief 获取选中的点云
         */
        std::vector<Cloud::Ptr> getSelectedClouds()
        {
            std::vector<Cloud::Ptr> clouds;
            for (auto& index : getSelectedIndexes())
                clouds.push_back(getCloud(index));
            return clouds;
        }

        /**
         * @brief 获取所有点云
         */
        std::vector<Cloud::Ptr> getAllClouds()
        {
            std::vector<Cloud::Ptr> clouds;
            for (auto& index : getAllIndexes())
            {
                clouds.push_back(getCloud(index));
            }
            return clouds;
        }

        /**
         * @brief 删除选中的点云
         */
        void removeSelectedClouds()
        {
            /**
             * @brief 使用降序排列是为了确保在删除点云时不会影响后续的删除操作,
             * 如果采用升序排列，那么在删除某个索引后，其后的索引会发生变化，这可能导致一些点云无法被正确删除。
             */
            for (auto& index : getSortedIndexes(DESCENDING, getSelectedIndexes()))
                removeCloud(index);
        }

        /**
         * @brief 删除所有点云
         */
        void removeAllClouds();

        /**
         * @brief 保存选中的点云
         */
        void saveSelectedClouds()
        {
            for (auto& index : getSelectedIndexes())
                saveCloud(index);
        }

        /**
         * @brief 合并选中的点云项目
         */
        void mergeSelectedClouds();

        /**
         * @brief 克隆选中的点云项目
         */
        void cloneSelectedClouds()
        {
            for (auto& index : getSelectedIndexes())
                cloneCloud(index);
        }

        /**
         * @brief 设置勾选点云,设置点云的选中状态
         */
        void setCloudChecked(const Cloud::Ptr& cloud, bool checked = true);

        /**
         * @brief 设置选中点云
         */
        void setCloudSelected(const Cloud::Ptr& cloud, bool selected = true);

        /**
         * @brief 显示进度条
         */
        void showProgressBar() {if (m_progress_bar) m_progress_bar->show(); }

        /**
         * @brief 关闭进度条
         */
        void closeProgressBar() {if (m_progress_bar) m_progress_bar->close(); }

    protected:
        /**
         * @brief 插入点云
         */
        void insertCloud(const Index& index, const Cloud::Ptr& cloud, bool selected = false);

        /**
         * @brief 移除该索引的点云
         */
        void removeCloud(const Index& index);

        /**
         * @brief 保存该索引的点云
         */
        void saveCloud(const Index& index);

        /**
         * @brief 克隆该索引的点云
         */
        void cloneCloud(const Index& index);

        /**
         * @brief 重命名该索引的点云
         */
        void renameCloud(const Index& index, const QString& name);

        /**
        * @brief 返回该索引的点云
        */
        Cloud::Ptr getCloud(const Index &index) {
            // 返回该索引对应的点云
            return m_cloud_vec[index.row][index.col];
        }

    signals:
        /**
         * @brief加载点云文件
         */
        void loadPointCloud(const QString& filename );

        /**
         * @brief 保存点云文件
         */
        void savePointCloud(const Cloud::Ptr& cloud, const QString& filename, bool isBinary);

        /**
         * @brief 删除点云的ID
         */
        void removedCloudId(const QString&);

    private slots:
        /**
         * @brief 加载点云文件的结果
         */
        void loadCloudResult(bool success, const Cloud::Ptr& cloud, float time);

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString& path, float time);

        /**
         * @brief 项目点击事件
         * @note 函数重写（覆盖），重写基类CustomTree中的itemClickedEvent函数
         *  在派生类中重写该槽函数，那么基类中的该槽函数是什么作用?
         */
        void itemClickedEvent(QTreeWidgetItem*, int);

        /**
         * @brief 项目选中改变事件
         * @note 函数重写（覆盖），重写基类CustomTree中的itemSelectionChangedEvent函数
         */
         void itemSelectionChangedEvent();

         void onFieldMappingRequested(const QList<ct::FieldInfo>& fields, QMap<QString, QString>& result);


    private:
        QString m_path;
        QThread m_thread;
        FileIO* m_fileio;
        QMenu* m_tree_menu;
        // 使用Cloud类型的智能指针的二维向量来实现树形结构
        std::vector<std::vector<Cloud::Ptr >> m_cloud_vec;
    };
}

#endif //CLOUDTOOL2_CLOUDTREE_H
