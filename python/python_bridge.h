#ifndef CLOUDTOOL2_PYTHON_BRIDGE_H
#define CLOUDTOOL2_PYTHON_BRIDGE_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QMap>
#include <QSet>
#include <vector>
#include "core/cloud.h"

namespace ct
{

/// Python→C++ 信号桥接
///
/// 供 Python 訡调用的方法内部只发射信号，不直接操作 UI。
/// 云注册表和引用持有均通过 QMutex 保护线程安全。
class PythonBridge : public QObject
{
    Q_OBJECT

public:
    explicit PythonBridge(QObject* parent = nullptr) : QObject(parent) {}

    // ================================================================
    // 日志与进度（在 Worker 稡调用，内部只 emit 信号）
    // ================================================================

    void log(int level, const QString& msg)              { emit signalLog(level, msg); }
    void showProgress(const QString& title)             { emit signalShowProgress(title); }
    void setProgress(int percent)                   { emit signalSetProgress(percent); }
    void closeProgress()                             { emit signalCloseProgress(); }

    // ================================================================
    // 视图控制
    // ================================================================

    void refreshView()     { emit signalRefreshView(); }
    void cloudChanged(const QString& id) { emit signalCloudChanged(id); }
    void resetCamera()       { emit signalResetCamera(); }
    void zoomToBounds()     { emit signalZoomToBounds(); }

    void setPointSize(int size)                     { emit signalSetPointSize(size); }
    void setBackgroundColor(float r, float g, float b) { emit signalSetBackgroundColor(r, g, b); }
    void showAxes(bool show)                       { emit signalShowAxes(show); }
    void showInfo(const QString& text)              { emit signalShowInfo(text); }

    // ================================================================
    // 云注册表（线程安全）
    // ================================================================

    /// 注册点云到查找表（主线程调用）
    void registerCloud(Cloud::Ptr cloud);

    /// 从查找表移除点云（主线程调用）
    void unregisterCloud(const QString& id);

    /// 按名称查找点云（Worker 稡调用，线程安全）
    Cloud::Ptr getCloud(const QString& name) const;

    // ================================================================
    // 引用持有（Worker 模调用，线程安全）
    // ================================================================

    /// 持有云引用（防止脚本执行期间被删除）
    void holdCloud(Cloud::Ptr cloud);

    /// 释放所有持有的云引用
    void releaseAllHeld();

    /// 通知主线程清除所有 in-use 标记
    void releaseAllInUse();

    // ================================================================
    // In-use 标记（Worker 模调用,通过信号通知 CloudTree）
    // ================================================================

    void markCloudInUse(const QString& id)  { emit signalMarkCloudInUse(id); }
    void unmarkCloudInUse(const QString& id){ emit signalUnmarkCloudInUse(id); }

signals:
    // 日志
    void signalLog(int level, QString message);

    void signalPrint(QString message);

    // 进度
    void signalShowProgress(QString title);
    void signalSetProgress(int percent);
    void signalCloseProgress();

    // 视图控制
    void signalCloudChanged(QString cloud_id);
    void signalResetCamera();
    void signalRefreshView();
    void signalZoomToBounds();

    // 颜色与视觉
    void signalSetPointSize(int size);
    void signalSetBackgroundColor(float r, float g, float b);
    void signalShowAxes(bool show);
    void signalShowInfo(QString text);
    void signalClearInfo();

    // 云注册表
    void signalCloudRegistered(QString name);
    void signalCloudUnregistered(QString id);

    // In-use 标记
    void signalMarkCloudInUse(QString cloud_id);
    void signalUnmarkCloudInUse(QString cloud_id);
    void signalReleaseAllInUse();

private:
    mutable QMutex m_cloud_mutex;
    QMap<QString, Cloud::Ptr> m_cloud_registry;  // name → Cloud
    std::vector<Cloud::Ptr> m_held_clouds;       // 脚本期间持有的引用
};

} // namespace ct

#endif // CLOUDTOOL2_PYTHON_BRIDGE_H
