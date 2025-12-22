#ifndef CLOUDTOOL2_CUSTOMDOCK_H
#define CLOUDTOOL2_CUSTOMDOCK_H

#include "base/cloudtree.h"
#include "base/cloudview.h"
#include "base/console.h"
#include "base/exports.h"

#include <QDockWidget>
#include <QMainWindow>
#include <QPushButton>
#include <map>
#include <set>

namespace ct
{
    class CT_EXPORT CustomDock : public QDockWidget
    {
        Q_OBJECT
    public:
        explicit CustomDock(QWidget* parent = nullptr) : QDockWidget(parent) {}

        ~CustomDock() {}

        void setCloudView(CloudView* cloudview) {m_cloudview = cloudview; }

        void setCloudTree(CloudTree* cloudtree) {m_cloudtree = cloudtree; }

        void setConsole(Console* console) {m_console = console; }

        virtual void init() {}

        virtual void reset() {}

        virtual void deinit() {}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        void printI(const QString& message) {m_console->print(LOG_INFO, message); }
        void printW(const QString& message) {m_console->print(LOG_WARNING, message); }
        void printE(const QString& message) {m_console->print(LOG_ERROR, message); }

        void closeEvent(QCloseEvent* event)
        {
            reset();
            deinit();
            return QDockWidget::closeEvent(event);
        }
    public:
        CloudView* m_cloudview;
        CloudTree* m_cloudtree;
        Console* m_console;
    };
    static std::map<QString, CustomDock*> registed_docks;
    static std::set<QString> left_label;
    static std::set<QString> right_label;
    static std::map<QString, bool> docks_visible;

    /**
     * @brief 创建停靠窗口
     * @param parent 主窗口,停靠的窗口将被添加到这个主窗口中
     * @param label 窗口标签， 窗口停靠的标签，用于标识和管理
     * @param area 停靠区域  left/right/top/bottom， 默认为左侧停靠区域
     * @param dock 指向另一个窗口的指针，用于合并停靠窗口，
     */
    template <class T>
    void createDock(QMainWindow* parent, const QString& label, CloudView* cloudview = nullptr,
                   CloudTree* cloudtree = nullptr, Console* console = nullptr,
                   Qt::DockWidgetArea area = Qt::LeftDockWidgetArea, QDockWidget* dock = nullptr)
    {
        // 如果主窗口为空，直接返回
        if (parent == nullptr) return;
        // 如果label对应的停靠窗口尚未注册，在registed_docks中注册一个空指针
        if (registed_docks.find(label) == registed_docks.end())  //register dock
            registed_docks[label] = nullptr;
        // 如果label对应的停靠窗口指针为空，
        if (registed_docks.find(label)->second == nullptr)  // creat new dock
        {
            // 创建一个新的停靠窗口
            registed_docks[label] = new T(parent);
            if (cloudview)
                registed_docks[label]->setCloudView(cloudview);
            if (cloudtree)
                registed_docks[label]->setCloudTree(cloudtree);
            if (console)
                registed_docks[label]->setConsole(console);
            // 设置停靠窗口在关闭时自动删除
            registed_docks[label]->setAttribute(Qt::WA_DeleteOnClose);
            // 调用init初始化停靠窗口
            registed_docks[label]->init();
            // visibilityChanged信号会传递一个bool参数给匿名函数，表示当前QDockWidget的可见状态，当前可见时state为true，反之为false
            // 匿名函数中反转这个状态。也就是说，docks_visible 会记录停靠窗口是隐藏的状态为 true，而可见的状态为 false。
            QObject::connect(registed_docks[label], &QDockWidget::visibilityChanged, [=](bool state)
                            {docks_visible[label] = !state; });
            // 当DockWidget被销毁时，注册表中对应的指针被设置为nullptr,避免了悬空指针的问题
            QObject::connect(registed_docks[label], &QDockWidget::destroyed, [=]
                            { registed_docks[label] = nullptr;});
            parent->addDockWidget(area, registed_docks[label]);
            // 如果指定的区域是左侧停靠，将当前标签插入到 left_label 集合中，否则插入到right_label 集合
            if (area == Qt::LeftDockWidgetArea)
                left_label.insert(label);
            else
                right_label.insert(label);
            // 如果指定的现有停靠窗口为空，
            if (dock == nullptr)
                for (auto& dock : registed_docks)
                {
                    // left
                    // 首先检查当前遍历的停靠窗口是否不是正在创建的窗口（通过比较标签label来判定）。
                    // 接着，它检查该窗口是否已经注册为左侧停靠窗口（通过left_label.count(dock.first) > 0）以及它是否不为空
                    if (dock.first != label && (left_label.count(dock.first) > 0) &&
                        area == Qt::LeftDockWidgetArea && dock.second != nullptr)
                    {
                        // 调用tabifyDockWidget方法将新创建的停靠窗口与现有的左侧停靠窗口并排显示
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                    // right
                    if (dock.first != label && (right_label.count(dock.first) > 0) &&
                        area == Qt::RightDockWidgetArea && dock.second != nullptr)
                    {
                        parent->tabifyDockWidget(dock.second, registed_docks[label]);
                        break;
                    }
                }
            else
                // 如果传入的dock参数不为nullptr，则直接将新创建的停靠窗口与指定的现有停靠窗口进行分组
                parent->tabifyDockWidget(dock, registed_docks[label]);
            // 设置窗口的可见性
            registed_docks[label]->setVisible(true);
            // 将指定的窗口提升至最高层，确保在其他窗口之上
            registed_docks[label]->raise();
        }
        else // update dock
        {
            if (docks_visible.find(label) == docks_visible.end()) return;
            if (docks_visible.find(label)->second)
                registed_docks[label]->raise();
            else
            {
                registed_docks[label]->close();
                registed_docks.erase(label);
            }
        }
    }

    /**
     * @brief 获取窗口指针
     */
    template <class T>
    T* getDock(const QString& label)
    {
        if (registed_docks.find(label) == registed_docks.end())
            return nullptr;
        else
            return (T*)registed_docks.find(label)->second;
    }
}
#endif //CLOUDTOOL2_CUSTOMDOCK_H
