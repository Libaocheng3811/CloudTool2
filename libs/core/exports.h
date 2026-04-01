#ifndef CLOUDTOOL2_EXPORTS_H
#define CLOUDTOOL2_EXPORTS_H

#define BOOST_ALLOW_DEPRECATED_HEADERS

// CT_EXPORT 用于在创建可共享的库时导出类、函数或其他符号
// 如果 CT_BUILDING_CORE 被定义（库构建时），CT_EXPORT 为导出符号
// 否则（使用库时），CT_EXPORT 为导入符号

#if defined(_WIN32) || defined(_WIN64)
    #ifdef CT_BUILDING_CORE
        #define CT_EXPORT __declspec(dllexport)
    #else
        #define CT_EXPORT __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define CT_EXPORT __attribute__((visibility("default")))
#else
    #define CT_EXPORT
#endif

#endif //CLOUDTOOL2_EXPORTS_H
