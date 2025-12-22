#include "base/fileio.h"
#include "pcl/filters/filter.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/ifs_io.h"
#include "pcl/io/obj_io.h"

namespace ct
{
    void FileIO::loadPointCloud(const QString &filename)
    {
        TicToc time;
        // 开始计时
        time.tic();
        Cloud::Ptr cloud(new Cloud);
        QFileInfo fileInfo(filename);
        int result = -1;
        if (fileInfo.suffix() == "pcd")
            result = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), *cloud);
        else if (fileInfo.suffix() == "ply")
            result = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), *cloud);
        else if (fileInfo.suffix() == "obj")
            result = pcl::io::loadOBJFile(filename.toLocal8Bit().toStdString(), *cloud);
        else if (fileInfo.suffix() == "ifs")
            result = pcl::io::loadIFSFile(filename.toLocal8Bit().toStdString(), *cloud);
        // 不是以上文件类型，打开失败
        else
        {
            emit loadCloudResult(false, cloud, time.toc());
            return;
        }

        // 是以上文件类型，但是打开失败
        if (result == -1)
        {
            emit loadCloudResult(false, cloud, time.toc());
            return;
        }
        cloud->is_dense = false;
        std::vector<int> indices; // 用于存储被移除的点的索引
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        cloud->setId(fileInfo.baseName());
        // 设置点云文件信息
        cloud->setInfo(fileInfo);
        // 更新点云
        cloud->update();
        emit loadCloudResult(true, cloud, time.toc());
    }

    void FileIO::savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary)
    {
        TicToc time;
        time.tic();
        QFileInfo fileInfo(filename);
        int result = -1;
        if (fileInfo.suffix() == "pcd")
            result = pcl::io::savePCDFile(filename.toLocal8Bit().toStdString(), *cloud, isBinary);
        else if (fileInfo.suffix() == "ply")
            result = pcl::io::savePLYFile(filename.toLocal8Bit().toStdString(), *cloud, isBinary);
        // 如果文件后缀既不是“pcd”也不是“ply”，那么就将文件名修改为以“.ply”结尾，
        else
        {
            result =pcl::io::savePLYFile(QString(filename + ".ply").toLocal8Bit().toStdString(), *cloud, isBinary);
        }

        if (result == -1)
            emit saveCloudResult(false, filename,time.toc());
        else
            emit saveCloudResult(true, filename, time.toc());
    }
}
