#include "python_worker.h"
#include "python_bridge.h"

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
#undef slots
#include <pybind11/embed.h>
#include <pybind11/eval.h>
#include <iostream>

namespace py = pybind11;

namespace ct
{

PythonWorker::PythonWorker(PythonBridge* bridge, QObject* parent)
    : QThread(parent), m_bridge(bridge)
{
}

void PythonWorker::execScript(const QString& script, const QString& filename)
{
    if (m_busy.load()) return;

    m_script = script;
    m_filename = filename.isEmpty() ? "<script>" : filename;
    m_is_file = false;
    m_cancel_flag.store(false);

    start();
}

void PythonWorker::execFile(const QString& filepath)
{
    if (m_busy.load()) return;

    m_filename = filepath;
    m_is_file = true;
    m_cancel_flag.store(false);

    start();
}

void PythonWorker::cancel()
{
    m_cancel_flag.store(true);
}

void PythonWorker::run()
{
    m_busy.store(true);
    emit scriptStarted();

    executePythonCode();

    m_busy.store(false);
}

void PythonWorker::executePythonCode()
{
    // 获取 GIL
    PyGILState_STATE gstate = PyGILState_Ensure();

    try {
        if (m_cancel_flag.load()) {
            emit scriptFinished(false, "Script canceled before execution");
            PyGILState_Release(gstate);
            return;
        }

        if (m_is_file) {
            // 执行文件
            py::eval_file(m_filename.toStdString());
        } else {
            // 执行脚本字符串
            py::exec(m_script.toStdString(), py::globals(), py::dict());
        }

        // 检查是否被取消
        if (m_cancel_flag.load()) {
            emit m_bridge->signalLog(1, "Script execution was interrupted");
        }

        emit scriptFinished(true, "");

    } catch (py::error_already_set& e) {
        QString error = QString::fromStdString(e.what());
        std::cerr << "[PythonWorker] Error: " << error.toStdString() << std::endl;
        emit scriptFinished(false, error);

    } catch (const std::exception& e) {
        QString error = QString::fromStdString(e.what());
        std::cerr << "[PythonWorker] Error: " << error.toStdString() << std::endl;
        emit scriptFinished(false, error);

    } catch (...) {
        emit scriptFinished(false, "Unknown error during script execution");
    }

    // 释放 GIL
    PyGILState_Release(gstate);
}

} // namespace ct
