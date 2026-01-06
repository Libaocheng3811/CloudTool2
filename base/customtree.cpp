#include "base/customtree.h"

namespace ct
{
    CustomTree::CustomTree(QWidget *parent)
        : QTreeWidget(parent),
        m_cloudview(nullptr),
        m_console(nullptr),
        m_table(nullptr)
    {
        /** QAbstractItemView 是 Qt 模型/视图框架中的一个抽象基类，提供了显示和编辑模型（Model）中数据的通用接口。
         *  setSelectionMode 函数接受一个 QAbstractItemView::SelectionMode 枚举值，定义了选择行为。
         *  QAbstractItemView::SingleSelection表示单选模式，QAbstractItemView::MultiSelection表示多选模式
         *  QAbstractItemView::ExtendedSelection表示扩展模式
         */
        this->setSelectionMode(QAbstractItemView::ExtendedSelection);

        // 信号itemSelectionChanged和itemClicked继承自QTreeWidget
        connect(this, &CustomTree::itemSelectionChanged, this, &CustomTree::itemSelectionChangedEvent);
        connect(this, &CustomTree::itemClicked, this, &CustomTree::itemClickedEvent);
        this->setAcceptDrops(true);
    }

    std::vector<Index> CustomTree::getSelectedIndexes()
    {
        // Index类型向量用来存储被选中的节点的索引
        std::vector<Index> indexes;
        // selectedItems()获取被选中的节点
        auto items = selectedItems();
        for (auto& i : items)
        {
            if (i->parent() != nullptr)
                indexes.push_back(Index(indexOfTopLevelItem(i->parent()), i->parent()->indexOfChild(i)));
        }
        return indexes;
    }

    std::vector<Index> CustomTree::getClickedIndexes(QTreeWidgetItem *item)
    {
        std::vector<Index> indexes;
        // 判断节点有无父节点（顶层节点没有父节点）
        // 如果item是顶级项，遍历它的所有子项，并为每个子项创建一个Index对象,意为可添加多个子项index
        // indexOfTopLevelItem(item)是QTreeWidget类（树形视图部件）中的一个方法。
        // 它用于返回给定顶级项目（QTreeWidgetItem类型）在树形部件的顶级项目列表中的索引。如果在顶级项目列表中找不到该项目，则返回 - 1。
        if (item->parent() == nullptr)
            for (int i = 0; i < item->childCount(); ++i)
                indexes.push_back(Index(indexOfTopLevelItem(item), i));
        // 如果是非顶级项，只添加一个index对象，indexOfChild(item)通常用于在某个父项（parent item）的子项列表中查找给定子项的索引位置。
        else
            indexes.push_back(Index(indexOfTopLevelItem(item->parent()),item->parent()->indexOfChild(item)));

        return indexes;
    }

    std::vector<Index> CustomTree::getAllIndexes()
    {
        std::vector<Index> indexes;
        for (int i = 0; i < topLevelItemCount(); i++)
        {
            for (int j = 0; j < topLevelItem(i)->childCount(); j++)
                indexes.push_back(Index(i, j));
        }
        return indexes;
    }

    bool CustomTree::indexIsValid(const ct::Index &index)
    {
        // 检查索引行大于-1且小于顶层节点数量，索引列大于-1且小于它顶层节点的子节点数量
        return (index.row > -1) && (index.row < topLevelItemCount()) &&
                (index.col > -1) && (index.col < topLevelItem(index.row)->childCount());
    }

    void CustomTree::addItem(const Index& index, const QString& parent_id, const QString& child_id, bool selected)
    {
        // 行索引无效，直接在根目录尾部添加新节点
        if (index.row <= -1 || index.row > topLevelItemCount()) // parent item
        {
            QTreeWidgetItem* parent = new QTreeWidgetItem(this);
            QTreeWidgetItem* child = new QTreeWidgetItem(parent);
            parent->setText(0, parent_id);
            parent->setIcon(0, m_parent_icon);
            parent->setCheckState(0, Qt::Checked);
            child->setText(0, child_id);
            child->setIcon(0, m_child_icon);
            child->setCheckState(0, Qt::Checked);
            parent->addChild(child);

            // addTopLevelItem 用于在 QTreeWidget 中添加一个新的顶级项,parent就是要添加的新项
            /**
             * @brief 添加QTreeWidgetItem节点
             * 这里就是实际关联控件添加节点，因为CloudTree类继承自CustomTree，而CustomTree又继承自QTreeWidget,
             */

            addTopLevelItem(parent);
            expandItem(parent);
            if (selected)
                setCurrentItem(child);
        }
        else // child
        {
            QTreeWidgetItem* parent = topLevelItem(index.row);
            QTreeWidgetItem* child = new QTreeWidgetItem(parent);
            child->setIcon(0, m_child_icon);
            child->setText(0, child_id);
            child->setCheckState(0, Qt::Checked);
            if ((index.col > -1) && (index.col < topLevelItem(index.row)->childCount()))
                parent->insertChild(0, child);
            else
                parent->addChild(child);
            // expandItem 用于展开指定项的所有子项。如果项已经展开，则此函数没有效果。
            expandItem(parent);
            if (selected)
                setCurrentItem(child);
        }
    }

    void CustomTree::removeItem(const ct::Index &index)
    {
        if (!indexIsValid(index))
            return;
        // 如果顶层项只有一个子项，那么删除顶层项及其唯一的子项
        // takeTopLevelItem 是 QTreeWidget 的成员函数，作用是删除指定行的顶层项
        if (topLevelItem(index.row)->childCount() == 1)
            takeTopLevelItem(index.row);
        // 如果顶层项有多个子项，调用顶层项的方法takeChild删除指定的子项
        // takeChild 方法从其父项中删除子项，并返回删除的子项
        else
            topLevelItem(index.row)->takeChild(index.col);
    }

    QTreeWidgetItem* CustomTree::item(const ct::Index &index)
    {
        // 如果索引值无效，返回空指针
        if (!indexIsValid(index))
            return nullptr;
        // 判断是顶层项还是子项，如果col为-1，说明是顶层项目，否则为子项
        if (index.col == -1)
            return topLevelItem(index.row);
        else
            return topLevelItem(index.row)->child(index.col);
    }

    Index CustomTree::index(const QString& text)
    {
        for (int i = 0; i < topLevelItemCount(); i++)
            for (int j = 0; j < topLevelItem(i)->childCount(); j++)
                if (topLevelItem(i)->child(j)->text(0) == text)
                    return Index(i, j);
        return Index(-1, -1);
    }

    void CustomTree::setItemChecked(const ct::Index &index, bool checked)
    {
        if (!indexIsValid(index))
            return;
        QTreeWidgetItem* item = topLevelItem(index.row)->child(index.col);
        if (checked)
            item->setCheckState(0, Qt::Checked);
        else
            item->setCheckState(0, Qt::Unchecked);
        itemClickedEvent(item);
    }

    std::vector<Index> CustomTree::getSortedIndexes(ct::sort_type type, const std::vector<Index> &indexes)
    {
        std::vector<Index> res;
        /**
         * 这是std::sort函数的调用，它是一个标准库算法，用于对序列进行排序。函数接受三个参数：
         * 第一个参数是序列的开始迭代器，res.begin()返回指向res第一个元素的迭代器。
         * 第二个参数是序列的结束迭代器，res.end()返回一个指向res末尾的迭代器。
         * 第三个参数是一个自定义的比较函数或函数对象，用于确定序列中两个元素的顺序。
         */
        switch (type)
        {
            case ASCENDING:
                res = indexes;
                /**
                 * @brief Lambda 表达式在 C++ 中是一种可以捕获并使用周围作用域中变量的匿名函数
                 * @param Index& a, Index& b 是lambda的参数，每次迭代都会从容器中取出两个元素，并传递给 Lambda 表达式作为参数
                 * std::sort 函数需要一个比较函数，其作用是确定两个元素之间的顺序关系。
                 * 这个比较函数必须接受两个参数，并返回一个布尔值。这个布尔值告诉 std::sort 函数如何根据比较结果来排序元素。
                 */
                // Index& a, Index& b是lambda的参数
                // -> bool指定了lambda的返回类型为bool
                std::sort(res.begin(), res.end(), [](Index& a, Index& b) -> bool
                    {return (a.row < b.row) || (a.row == b.row && a.col < b.col) ? true : false; });
                break;
            case DESCENDING:
                res = indexes;
                std::sort(res.begin(), res.end(), [](Index& a, Index& b) -> bool
                    {return (a.row > b.row) || (a.row == b.row && a.col > b.col) ? true : false; });
                break;
            case PARENTFIRST:
                for (auto& i : indexes)
                {
                    // 当前index对象不是顶层项时，
                    if (i.col != -1)
                    {
                        // 查找当前索引的父项是否在索引中，如果不在，就将当前项的索引添加到结果中
                        if (std::find(indexes.begin(), indexes.end(), Index(i.row, -1)) == indexes.end())
                            res.push_back(i);
                    }
                    // 当前index对象是顶层项
                    else
                    {
                        // 获取当前项，并循环将当前项的所有子项添加到结果中
                        QTreeWidgetItem* parent = topLevelItem(i.row);
                        for (int j = 0; j < parent->childCount(); j++)
                            res.push_back(Index(i.row, j));
                    }
                }
                break;
            case CHILDFIRST:
                for (auto& i : indexes)
                    if (i.col != -1)
                        res.push_back(i);
                break;
        }
        return res;
    }

    void CustomTree::itemSelectionChangedEvent()
    {
        // 用于获取当前选择的项的列表
        // selectedItems() 是 QTreeWidget 或 QListWidget 类的一个成员函数，用于返回当前选中的所有项的列表。
        // 该函数返回一个 QList<QTreeWidgetItem*> 或 QList<QListWidgetItem*>，
        auto items = selectedItems();
        for (auto& item : items)
        {
            // 如果节点是顶层节点，就设置它的所有子节点为选中状态
            if (item->parent() == nullptr)
                for (int i = 0; i < item->childCount(); i++)
                    item->child(i)->setSelected(true);
            // 如果不是顶层节点，判断它的子节点是否被全部选中，如果子节点全被选中，设置父节点isSelected()为选中状态
            else
            {
                int i, cout = item->parent()->childCount();
                for (i = 0; i < cout; i++)
                    if (!item->parent()->child(i)->isSelected())
                        break;
                if (i == cout)
                    item->parent()->setSelected(true);
            }
        }
    }

    void CustomTree::itemClickedEvent(QTreeWidgetItem* item, int)
    {
        // 如果节点没有父节点，也就是顶层节点
        if (item->parent() == nullptr)
        {
            // 根据父节点的复选框状态设置子节点的复选框状态，checkState(0)返回该项在第0列的复选框状态
            for (int i = 0; i < item->childCount(); i++)
            {
                if (item->checkState(0) == Qt::Checked)
                    item->child(i)->setCheckState(0, Qt::Checked);
                else if (item->checkState(0) == Qt::Unchecked)
                    item->child(i)->setCheckState(0, Qt::Unchecked);
            }
        }
        // 否则，如果节点不是顶层节点
        else
        {
            QTreeWidgetItem* parent = item->parent();
            // 如果子节点复选框处于选中状态，检查其父节点下的所有子节点的选中状态，
            // 如果所有子节点都处于选中状态，则设置父节点为选中状态；如果子节点中有未被选中的节点，设置父节点为部分选中状态PartiallyChecked
            if (item->checkState(0) == Qt::Checked)
            {
                int i, cout = parent->childCount();
                for (i = 0; i < cout; i++)
                    if (parent->child(i)->checkState(0) == Qt::Unchecked)
                        break;
                if (i == cout)
                    parent->setCheckState(0, Qt::Checked);
                else
                    parent->setCheckState(0, Qt::PartiallyChecked);
            }
            // 如果子节点复选框处于未选中状态，则与上述反之，子节点全未选中就设置父节点为未选中状态，否则为部分选中状态
            else
            {
                int i, cout = parent->childCount();
                for (i = 0; i < cout; i++)
                    if (parent->child(i)->checkState(0) == Qt::Checked)
                        break;
                if (i == cout)
                    parent->setCheckState(0, Qt::Unchecked);
                else
                    parent->setCheckState(0, Qt::PartiallyChecked);
            }
        }
    }
}
