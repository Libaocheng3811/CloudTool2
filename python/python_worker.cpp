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

    // 向 Worker 的 Python 线程注入 KeyboardInterrupt 异常
    unsigned long tid = m_py_thread_id.load();
    if (tid != 0) {
        PyThreadState_SetAsyncExc(tid, PyExc_KeyboardInterrupt);
    }
}

void PythonWorker::run()
{
    m_busy.store(true);
    m_py_thread_id.store(0);
    emit scriptStarted();

    executePythonCode();

    // 脚本执行完毕，释放所有持有的云引用并通知 UI
    if (m_bridge) {
        m_bridge->releaseAllHeld();
        m_bridge->releaseAllInUse();
    }

    m_busy.store(false);
}

void PythonWorker::executePythonCode()
{
    // 获取 GIL
    PyGILState_STATE gstate = PyGILState_Ensure();

    // 记录当前线程在 Python 解释器中的 ID（用于 cancel 注入异常）
    m_py_thread_id.store(PyThreadState_Get()->thread_id);

    try {
        if (m_cancel_flag.load()) {
            emit scriptFinished(false, "Script canceled before execution");
            m_py_thread_id.store(0);
            PyGILState_Release(gstate);
            return;
        }

        if (m_is_file) {
            py::eval_file(m_filename.toStdString());
        } else {
            py::exec(m_script.toStdString(), py::globals(), py::dict());
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

    m_py_thread_id.store(0);
    // 释放 GIL
    PyGILState_Release(gstate);
}

} // namespace ct
