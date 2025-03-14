#ifndef CLOUDTOOL2_FILEIO_H
#define CLOUDTOOL2_FILEIO_H

#include "base/cloud.h"
#include "base/exports.h"

namespace ct
{
    class CT_EXPORT FileIO :public QObject
    {
        Q_OBJECT
    public:
        explicit FileIO(QObject *parent = nullptr) : QObject(parent) {}

        // 信号不需实现，只需声明即可
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

    };
}



#endif //CLOUDTOOL2_FILEIO_H
