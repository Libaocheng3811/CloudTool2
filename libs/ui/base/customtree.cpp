#include "customtree.h"

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

    QList<QTreeWidgetItem*> CustomTree::getSelectedItems() {
        return this->selectedItems();
    }

    void CustomTree::getAllChildItems(QTreeWidgetItem* parent, QList<QTreeWidgetItem*>& lists){
        if (!parent) return;
        for (int i = 0; i < parent->childCount(); ++i){
            QTreeWidgetItem* child = parent->child(i);
            lists.append(child);
            getAllChildItems(child, lists); // 递归获取所有子节点
        }
    }

    void CustomTree::setItemAndChildrenCheckState(QTreeWidgetItem *item, Qt::CheckState state) {
        if (!item) return;
        const bool wasBlocked = this->blockSignals(true);

        item->setCheckState(0, state);
        for (int i = 0; i < item->childCount(); ++i){
            setItemAndChildrenCheckState(item->child(i), state);
        }
        this->blockSignals(wasBlocked);
    }

    QTreeWidgetItem* CustomTree::addItem(QTreeWidgetItem *parent, const QString &text, bool selected) {
        QTreeWidgetItem* new_item;
        if (parent == nullptr){
            // 如果是根节点
            new_item = new QTreeWidgetItem(this);
            new_item->setIcon(0, m_parent_icon);
        }
        else{
            new_item = new QTreeWidgetItem(parent);
            // 子节点根据是否有子节点决定图标，这里默认用子图标，逻辑可扩展
            new_item->setIcon(0, m_child_icon);
        }
        new_item->setText(0, text);
        new_item->setCheckState(0, Qt::Checked);

        // 展开父节点
        if (parent) parent->setExpanded(true);
        else this->expandItem(new_item);

        if (selected){
            this->clearSelection();
            new_item->setSelected(true);
        }
        return new_item;
    }

    void CustomTree::removeItem(QTreeWidgetItem *item) {
        if (item == nullptr) return;

        //如果有父节点，从父节点移除，否则从TreeWidget移除
        if (item->parent()){
            item->parent()->removeChild(item);
        }
        else {
            int idx = indexOfTopLevelItem(item);
            this->takeTopLevelItem(idx);
        }
        delete item;
    }

    void CustomTree::itemSelectionChangedEvent() {}

    void CustomTree::itemClickedEvent(QTreeWidgetItem* item, int) {}
}
