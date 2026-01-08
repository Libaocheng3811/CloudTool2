#ifndef CLOUDTOOL2_FILEIO_H
#define CLOUDTOOL2_FILEIO_H

#include "base/cloud.h"
#include "base/exports.h"
#include "base/txtimportdialog.h"
#include "base/txtexportdialog.h"

#include <atomic>

namespace ct
{
    struct FieldInfo{
        QString name;
        QString type;
    };

    class CT_EXPORT FileIO : public QObject
    {
        Q_OBJECT
    public:
        explicit FileIO(QObject *parent = nullptr) : QObject(parent) {}

    signals:
        /**
         * @brief 加载点云文件的结果
         * @note const Cloud::Ptr &cloud 表示参数cloud是对Cloud::Ptr 类型的智能指针的引用，且这个引用是常量
         */
        void loadCloudResult(bool success, const Cloud::Ptr &cloud, float time);

        /**
         * @brief 保存点云文件的结果
         */
        void saveCloudResult(bool success, const QString &filename, float time);

        /**
        * @brief 请求UI显示映射对话框 (阻塞式)
        * @param fields 文件中探测到的字段列表
        * @param result 用户选择的映射结果 (引用传出)
        */
        void requestFieldMapping(const QList<ct::FieldInfo>& fields, QMap<QString, QString>& result);

        /**
         * @brief 显示映射对话框 (阻塞式)
         * @param preview_lines 文件中探测到的字段列表
         * @param params 用户选择的映射结果 (引用传入)
         */
         void requestTxtImportSetup(const QStringList& preview_lines, ct::TxtImportParams& params);

         /**
          * @brief 显示导出对话框 (阻塞式)
          * @param available_fields 告诉UI 可用的字段列表
          * @param params 接收用户配置
          */
         void requestTxtExportSetup(const QStringList& available_fields, ct::TxtExportParams& params);

         /**
          * @brief 进度信号
          */
         void progress(int percent);

         /**
          * @brief 请求全局偏移设置
          * @param bounding_min 原始数据的最小点 (用于显示)
          * @param suggested_shift 建议的偏移值
          * @param is_skipped 用户是否选择跳过大坐标偏移，使用大坐标显示
          */
         void requestGlobalShift(const Eigen::Vector3d& bounding_min, Eigen::Vector3d& suggested_shift, bool& is_skipped);

    public slots:
        /**
         * @brief 加载点云文件
         * @param filename 点云文件路径
         */
        void loadPointCloud(const QString &filename);

        /**
         * @brief 保存点云文件
         * @param cloud 点云数据
         * @param filename 保存的文件路径
         */
        void savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary);

        /**
         * @brief 取消当前操作
         */
        void cancel() { m_is_canceled = true;};

    private:
        bool loadLAS(const QString &filename, Cloud::Ptr &cloud);
        bool loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud); // 支持自定义字段
        bool loadTXT(const QString &filename, Cloud::Ptr &cloud); // 支持交互
        bool loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud); // OBJ, IFS 等标准格式

        bool saveLAS(const Cloud::Ptr &cloud, const QString &filename);
        bool saveTXT(const Cloud::Ptr &cloud, const QString &filename);
        bool savePCL(const Cloud::Ptr &cloud, const QString &filename, bool isBinary);

    private:
        std::atomic<bool> m_is_canceled{false};
    };
}



#endif //CLOUDTOOL2_FILEIO_H
