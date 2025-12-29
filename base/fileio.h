#ifndef CLOUDTOOL2_FILEIO_H
#define CLOUDTOOL2_FILEIO_H

#include "base/cloud.h"
#include "base/exports.h"
#include "base/txtimportdialog.h"

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

    private:
        bool loadLAS(const QString &filename, Cloud::Ptr &cloud);
        bool loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud); // 支持自定义字段
        bool loadTXT(const QString &filename, Cloud::Ptr &cloud); // 支持交互
        bool loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud); // OBJ, IFS 等标准格式
    };
}



#endif //CLOUDTOOL2_FILEIO_H
