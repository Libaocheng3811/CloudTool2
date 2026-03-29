#ifndef CLOUDTOOL2_PYTHON_BRIDGE_H
#define CLOUDTOOL2_PYTHON_BRIDGE_H

#include <QObject>
#include <QString>

namespace ct
{

class PythonBridge : public QObject
{
    Q_OBJECT

public:
    explicit PythonBridge(QObject* parent = nullptr) : QObject(parent) {}

    // === 供 Python 调用的方法（在 Worker 线程中执行）===
    // 这些方法内部只发射信号，不直接操作 UI

    void log(int level, const QString& msg) { emit signalLog(level, msg); }
    void showProgress(const QString& title) { emit signalShowProgress(title); }
    void setProgress(int percent) { emit signalSetProgress(percent); }
    void closeProgress() { emit signalCloseProgress(); }

    void refreshView() { emit signalRefreshView(); }
    void cloudChanged(const QString& id) { emit signalCloudChanged(id); }
    void resetCamera() { emit signalResetCamera(); }
    void zoomToBounds() { emit signalZoomToBounds(); }

    void setPointSize(int size) { emit signalSetPointSize(size); }
    void setBackgroundColor(float r, float g, float b) { emit signalSetBackgroundColor(r, g, b); }
    void showAxes(bool show) { emit signalShowAxes(show); }
    void showInfo(const QString& text) { emit signalShowInfo(text); }

signals:
    // 日志
    void signalLog(int level, QString message);
    void signalPrint(QString message);  // Python print() 重定向

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
};

} // namespace ct

#endif // CLOUDTOOL2_PYTHON_BRIDGE_H
