#ifndef CLOUDTOOL2_PYTHON_BRIDGE_H
#define CLOUDTOOL2_PYTHON_BRIDGE_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QMap>
#include <vector>
#include "core/cloud.h"

namespace ct
{

/// Python→C++ 信号桥接
///
/// 供 Python 调用的方法内部只发射信号，不直接操作 UI。
/// 云注册表和引用持有均通过 QMutex 保护线程安全。
class PythonBridge : public QObject
{
    Q_OBJECT

public:
    explicit PythonBridge(QObject* parent = nullptr) : QObject(parent) {}

    // ================================================================
    // 日志与进度
    // ================================================================

    void log(int level, const QString& msg)              { emit signalLog(level, msg); }
    void showProgress(const QString& title)             { emit signalShowProgress(title); }
    void setProgress(int percent)                   { emit signalSetProgress(percent); }
    void closeProgress()                             { emit signalCloseProgress(); }

    // ================================================================
    // Python stdio 输出路由（Python Console 用）
    // ================================================================

    void printStdout(const QString& text)                { emit signalPrintStdout(text); }
    void printStderr(const QString& text)                { emit signalPrintStderr(text); }

    // ================================================================
    // 视图控制
    // ================================================================

    void refreshView()                                 { emit signalRefreshView(); }
    void cloudChanged(const QString& id)               { emit signalCloudChanged(id); }
    void resetCamera()                                 { emit signalResetCamera(); }
    void zoomToBounds()                               { emit signalZoomToBounds(); }
    void setAutoRender(bool enable)                   { emit signalSetAutoRender(enable); }
    void zoomToSelected()                             { emit signalZoomToSelected(); }

    void setTopView()                                 { emit signalSetTopView(); }
    void setFrontView()                               { emit signalSetFrontView(); }
    void setBackView()                                { emit signalSetBackView(); }
    void setLeftSideView()                            { emit signalSetLeftSideView(); }
    void setRightSideView()                           { emit signalSetRightSideView(); }
    void setBottomView()                              { emit signalSetBottomView(); }

    // ================================================================
    // 点云外观
    // ================================================================

    void setPointSize(const QString& id, float size)  { emit signalSetPointSize(id, size); }
    void setOpacity(const QString& id, float value)   { emit signalSetOpacity(id, value); }
    void setCloudColorRGB(const QString& id, float r, float g, float b)
                                                      { emit signalSetCloudColorRGB(id, r, g, b); }
    void setCloudColorByAxis(const QString& id, const QString& axis)
                                                      { emit signalSetCloudColorByAxis(id, axis); }
    void resetCloudColor(const QString& id)            { emit signalResetCloudColor(id); }
    void setCloudVisibility(const QString& id, bool v){ emit signalSetCloudVisibility(id, v); }

    // ================================================================
    // 场景外观
    // ================================================================

    void setBackgroundColor(float r, float g, float b){ emit signalSetBackgroundColor(r, g, b); }
    void resetBackgroundColor()                       { emit signalResetBackgroundColor(); }

    // ================================================================
    // 显示开关
    // ================================================================

    void showId(bool show)                           { emit signalShowId(show); }
    void showAxes(bool show)                         { emit signalShowAxes(show); }
    void showFPS(bool show)                          { emit signalShowFPS(show); }
    void showInfo(const QString& text)               { emit signalShowInfo(text); }
    void clearInfo()                                 { emit signalClearInfo(); }

    // ================================================================
    // 叠加物
    // ================================================================

    void addCube(float cx, float cy, float cz, float size, const QString& id)
                                                      { emit signalAddCube(cx, cy, cz, size, id); }
    void add3DLabel(const QString& text, float x, float y, float z, const QString& id)
                                                      { emit signalAdd3DLabel(text, x, y, z, id); }
    void removeShape(const QString& id)               { emit signalRemoveShape(id); }
    void removeAllShapes()                           { emit signalRemoveAllShapes(); }

    // ================================================================
    // 点云管理
    // ================================================================

    void insertCloud(Cloud::Ptr cloud)               { emit signalInsertCloud(cloud); }
    void removeSelectedClouds()                       { emit signalRemoveSelectedClouds(); }

    // ================================================================
    // 云注册表（线程安全）
    // ================================================================

    void registerCloud(Cloud::Ptr cloud);
    void unregisterCloud(const QString& id);
    Cloud::Ptr getCloud(const QString& name) const;

    // ================================================================
    // 引用持有 + In-use 标记
    // ================================================================

    void holdCloud(Cloud::Ptr cloud);
    void releaseAllHeld();
    void releaseAllInUse();
    void markCloudInUse(const QString& id)            { emit signalMarkCloudInUse(id); }
    void unmarkCloudInUse(const QString& id)          { emit signalUnmarkCloudInUse(id); }

signals:
    // 日志
    void signalLog(int level, QString message);
    void signalPrint(QString message);

    // Python stdio（Python Console 用）
    void signalPrintStdout(QString text);
    void signalPrintStderr(QString text);

    // 进度
    void signalShowProgress(QString title);
    void signalSetProgress(int percent);
    void signalCloseProgress();

    // 视图控制
    void signalCloudChanged(QString cloud_id);
    void signalResetCamera();
    void signalRefreshView();
    void signalZoomToBounds();
    void signalSetAutoRender(bool enable);
    void signalZoomToSelected();

    void signalSetTopView();
    void signalSetFrontView();
    void signalSetBackView();
    void signalSetLeftSideView();
    void signalSetRightSideView();
    void signalSetBottomView();

    // 点云外观
    void signalSetPointSize(QString id, float size);
    void signalSetOpacity(QString id, float value);
    void signalSetCloudColorRGB(QString id, float r, float g, float b);
    void signalSetCloudColorByAxis(QString id, QString axis);
    void signalResetCloudColor(QString id);
    void signalSetCloudVisibility(QString id, bool visible);

    // 场景外观
    void signalSetBackgroundColor(float r, float g, float b);
    void signalResetBackgroundColor();

    // 显示开关
    void signalShowId(bool show);
    void signalShowAxes(bool show);
    void signalShowFPS(bool show);
    void signalShowInfo(QString text);
    void signalClearInfo();

    // 叠加物
    void signalAddCube(float cx, float cy, float cz, float size, QString id);
    void signalAdd3DLabel(QString text, float x, float y, float z, QString id);
    void signalRemoveShape(QString id);
    void signalRemoveAllShapes();

    // 点云管理
    void signalInsertCloud(ct::Cloud::Ptr cloud);
    void signalRemoveSelectedClouds();

    // 注册表
    void signalCloudRegistered(QString name);
    void signalCloudUnregistered(QString id);

    // In-use
    void signalMarkCloudInUse(QString cloud_id);
    void signalUnmarkCloudInUse(QString cloud_id);
    void signalReleaseAllInUse();

private:
    mutable QMutex m_cloud_mutex;
    QMap<QString, Cloud::Ptr> m_cloud_registry;
    std::vector<Cloud::Ptr> m_held_clouds;
};

} // namespace ct

#endif // CLOUDTOOL2_PYTHON_BRIDGE_H
