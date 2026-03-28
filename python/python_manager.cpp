#include "python_manager.h"
#include "python_worker.h"

// Qt 的 <QObject> 定义了 slots 宏，与 Python 的 object.h 冲突
// 必须在包含 pybind11/Python 头文件之前取消该宏定义
#undef slots
#include <pybind11/embed.h>
#include <QCoreApplication>
#include <QDir>
#include <iostream>

namespace py = pybind11;

namespace ct
{

PythonManager& PythonManager::instance()
{
    static PythonManager mgr;
    return mgr;
}

PythonManager::PythonManager() = default;

PythonManager::~PythonManager()
{
    if (m_initialized)
        finalize();
}

void PythonManager::initialize()
{
    if (m_initialized) return;

    try {
        // 1. 启动 Python 解释器
        py::initialize_interpreter();

        // 2. 重定向 stdout/stderr 到控制台（嵌入式 Python 默认不连接控制台）
        redirectStdio();

        // 3. 添加搜索路径（脚本目录）
        addSearchPaths();

        // 4. 创建 Bridge 和 Worker
        m_bridge = new PythonBridge();
        m_worker = new PythonWorker(m_bridge);

        // 5. 主线程释放 GIL，从此主线程进入"无锁"状态
        //    专心处理 UI 事件和 VTK 渲染
        PyEval_SaveThread();

        m_initialized = true;

    } catch (const py::error_already_set& e) {
        std::cerr << "[PythonManager] Init failed: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[PythonManager] Init failed: " << e.what() << std::endl;
    }
}

void PythonManager::finalize()
{
    if (!m_initialized) return;

    // 1. 停止工作线程
    if (m_worker) {
        m_worker->cancel();
        m_worker->wait(3000);
        if (m_worker->isRunning())
            m_worker->terminate();
        delete m_worker;
        m_worker = nullptr;
    }

    // 2. 删除 Bridge
    delete m_bridge;
    m_bridge = nullptr;

    // 3. 不调用 Py_Finalize()
    //    NumPy/SciPy 等 C 扩展库在 Py_Finalize 时会触发段错误
    //    嵌入式 Python 桌面软件的业界惯例：依赖 OS 回收进程资源

    m_initialized = false;
}

void PythonManager::redirectStdio()
{
    // 嵌入式 Python 的 sys.stdout/stderr 默认为 None，print() 输出被丢弃
    // 用 os.write 直接写文件描述符，绕过所有缓冲，确保输出到控制台
    py::exec(R"(
import os, sys

class _ConsoleRedirect:
    def __init__(self, fd):
        self._fd = fd
    def write(self, text):
        if text:
            os.write(self._fd, text.encode('utf-8'))
    def flush(self):
        pass

sys.stdout = _ConsoleRedirect(1)
sys.stderr = _ConsoleRedirect(2)
)");
}

void PythonManager::addSearchPaths()
{
    // 添加应用程序目录下的 scripts/ 和 python/ 目录到 sys.path
    auto sys = py::module_::import("sys");
    auto paths = sys.attr("path").cast<py::list>();

    QString appDir = QCoreApplication::applicationDirPath();
    QStringList searchDirs = {
        appDir + "/scripts",
        appDir + "/python",
    };

    for (const auto& dir : searchDirs) {
        if (QDir(dir).exists()) {
            paths.append(dir.toStdString());
        }
    }
}

} // namespace ct
