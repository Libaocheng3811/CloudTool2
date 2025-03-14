#ifndef CLOUDTOOL2_EXPORTS_H
#define CLOUDTOOL2_EXPORTS_H

#include "QtCore/qglobal.h"
#define BOOST_ALLOW_DEPRECATED_HEADERS


// CT_EXPORT 是一个宏，通常用于在创建可共享的库时导出类、函数或其他符号
// 如果 CT_BUILD_LIBRARY 被定义（通常在库的构建过程中设置），CT_EXPORT 将使用 Q_DECL_EXPORT 宏来导出符号。
// 如果 CT_BUILD_LIBRARY 没有被定义（在使用库的应用程序中设置），CT_EXPORT 将使用 Q_DECL_IMPORT 宏来导入符号。
// Q_DECL_EXPORT 和 Q_DECL_IMPORT 是 Qt 提供的宏，用于处理 Windows 上的 DLL 和 Unix/Linux 上的 SO 文件的符号导出和导入。

#if defined(CT_LIBRARY)
#define CT_EXPORT Q_DECL_EXPORT
#else
#define CT_EXPORT Q_DECL_IMPORT
#endif

#endif //CLOUDTOOL2_EXPORTS_H
