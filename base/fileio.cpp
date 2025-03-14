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
        // pcl::console::TicToc 是PCL中的一个工具类，它用于测量代码的执行时间
        TicToc time;
        // 开始计时
        time.tic();
        // 将 new Cloud 创建的对象的指针包装进一个 shared_ptr 智能指针中，并赋值给变量 cloud。
        // Ptr是指向Cloud类对象的智能指针，
        // cloud就是指向Cloud对象的智能指针，用来存储从文件中提取的点云
        Cloud::Ptr cloud(new Cloud);
        // QFileInfo 类提供了一种方便的方式来访问文件的信息，而无需打开文件,提供了关于文件的详细信息，如文件名、路径、大小、权限等
        QFileInfo fileInfo(filename);
        int result = -1;
        // filename.toLocal8Bit().toStdString()是将QString类型转为string类型，这是loadPCDFile函数需要的类型
        // *cloud是解引用，提供对实际对象的引用，
        // pcl::io::loadPCDFile 函数的返回值是一个整数类型（int）。这个返回值通常用于指示文件保存操作的结果，通常 0 表示成功，-1 表示失败
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

        // remove NaN points
        // 移除 NaN 点是一个常见的预处理步骤，特别是在处理从传感器捕获的点云数据时.
        // is_dense 属性用来指示点云是否包含所有有效的点，没有任何空间被跳过。
        cloud->is_dense = false;
        std::vector<int> indices; // 用于存储被移除的点的索引
        // pcl::removeNaNFromPointCloud 是 PCL 中的一个函数，用于从点云中移除包含 NaN（非数字）值的点
        // 第一个和第二个参数都是 *cloud，表示输入点云和输出点云是同一个。这意味着函数会直接在原始点云上操作，移除 NaN 点。
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // cloud init
        // 设置点云id，baseName() 方法返回文件的基本名称，不包括路径和扩展名
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
            // pcl::io::savePCDFile 函数的返回值是一个整数类型（int）。这个返回值通常用于指示文件保存操作的结果，通常 0 表示成功，-1 表示失败
            // isBinary：一个布尔值，指示保存文件的格式。如果为 true，则以二进制格式保存点云数据；如果为 false，则以文本格式保存。
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
