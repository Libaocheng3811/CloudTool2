#ifndef CLOUDTOOL2_CUSTORMTREE_H
#define CLOUDTOOL2_CUSTORMTREE_H

#include "base/exports.h"

#include "base/cloudview.h"
#include "base/console.h"

#include <QTreeWidget>
#include <QTableWidget>
#include <QProgressBar>

namespace ct
{
    // 声明枚举变量sort_type，算作一种新的数据类型
    enum sort_type
    {
        ASCENDING, // 递增
        DESCENDING, // 递减
        PARENTFIRST, // 父节点优先
        CHILDFIRST // 子节点优先
    };

    /**
     * @brief 树形结构中的索引
     * 行索引：表示节点在哪一层，在逻辑树结构中，row 可以表示节点在父节点下的顺序。
     * 列索引：表示节点在行中的哪个位置，在逻辑树结构中，col 可以表示节点相对于其兄弟节点的位置。
     */
     // 在struct结构体中默认所有成员是public，而在类中默认所有成员是private
    struct Index
    {
        Index() {}
        Index(int r, int c) : row(r), col(c) {}

        // 运算符重载
        bool operator==(const Index& index) const
        {
            return (this->row == index.row) && (this->col == index.col);
        }
        bool operator!=(const Index& index) const
        {
            return !(*this == index);
        }
        int row = -1;
        int col = -1;
    };

    class CT_EXPORT CustomTree : public QTreeWidget
    {
        Q_OBJECT
    public:
        explicit CustomTree(QWidget* parent = nullptr);
        // 没有为一个类显式地写出析构函数，编译器会为你生成一个默认的析构函数

        /**
         * @brief 设置点云视图
         */
        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview;}

        /**
         * @brief 设置属性显示窗口
         */
        void setPropertiesTable(QTableWidget* table) {m_table = table;}

        /**
         * @brief 设置输出窗口
         */
        void setConsole(Console* console) {m_console = console;}

        /**
         * @brief 设置父类项目的图标
         */
        void setParentIcon(const QIcon& icon) {m_parent_icon = icon;}

        /**
         * @brief 设置子类项目的图标
         */
        void setChildIcon(const QIcon& icon) {m_child_icon = icon;}

    protected:
        /**
         * @brief 打印日志
         */
        void printI(const QString& message) {m_console->print(LOG_INFO, message);}
        void printW(const QString& message) {m_console->print(LOG_WARNING, message);}
        void printE(const QString& message) {m_console->print(LOG_ERROR, message);}

        /**
         * @brief 获取选中项目的索引
         */
        std::vector<Index> getSelectedIndexes();

        /**
         * @brief 获取勾选项目的索引
         */
        std::vector<Index> getCheckedIndexes();

        /**
        * @brief 获取点击项目的索引
        */
        std::vector<Index> getClickedIndexes(QTreeWidgetItem* item);

        /**
        * @brief 获取所有项目的索引
        */
        std::vector<Index> getAllIndexes();

        /**
        * @brief 返回索引是否有效
        */
        bool indexIsValid(const Index& index);

        /**
        * @brief 添加项目
        */
        void addItem(const Index& index, const QString& parent_id, const QString& child_id, bool selected = false);

        /**
        * @brief 移除项目
        */
        void removeItem(const Index& index);

        /**
        * @brief 返回该索引的项目
        */
        QTreeWidgetItem* item(const Index& index);

        /**
        * @brief 返回该项目名称的索引
        */
        Index index(const QString& text);

        /**
        * @brief 设置项目是否勾选
        */
        void setItemChecked(const Index& index, bool checked);

        /**
        * @brief 按照规则排序索引
        */
        std::vector<Index> getSortedIndexes(sort_type type, const std::vector<Index>& indexes);

    private slots:

        /**
        * @brief 项目选中改变事件
        */
        void itemSelectionChangedEvent();

        /**
        * @brief 项目点击事件
        */
        void itemClickedEvent(QTreeWidgetItem*, int i = 0);

    public:
        CloudView* m_cloudview;
        Console* m_console;
        QTableWidget* m_table;

    private:
        QIcon m_parent_icon;
        QIcon m_child_icon;

    };
}


#endif //CLOUDTOOL2_CUSTORMTREE_H
