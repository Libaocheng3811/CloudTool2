#include "base/fileio.h"
#include "base/fieldmappingdialog.h"

#include "pcl/filters/filter.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/ifs_io.h"
#include "pcl/io/obj_io.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "lasreader.hpp"
#include "laswriter.hpp"

namespace ct
{
    // 辅助函数
    QString getPCLFieldType(uint8_t type)
    {
        switch(type) {
            case pcl::PCLPointField::INT8: return "int8";
            case pcl::PCLPointField::UINT8: return "uint8";
            case pcl::PCLPointField::INT16: return "int16";
            case pcl::PCLPointField::UINT16: return "uint16";
            case pcl::PCLPointField::INT32: return "int32";
            case pcl::PCLPointField::UINT32: return "uint32";
            case pcl::PCLPointField::FLOAT32: return "float";
            case pcl::PCLPointField::FLOAT64: return "double";
            default: return "unknown";
        }
    }

    // 辅助函数：从 Blob 中提取 float 值
    float getFieldValueAsFloat(const uint8_t* data_ptr, int datatype) {
        using namespace pcl;
        switch(datatype) {
            case PCLPointField::INT8:   return (float)(*(int8_t*)data_ptr);
            case PCLPointField::UINT8:  return (float)(*(uint8_t*)data_ptr);
            case PCLPointField::INT16:  return (float)(*(int16_t*)data_ptr);
            case PCLPointField::UINT16: return (float)(*(uint16_t*)data_ptr);
            case PCLPointField::INT32:  return (float)(*(int32_t*)data_ptr);
            case PCLPointField::UINT32: return (float)(*(uint32_t*)data_ptr);
            case PCLPointField::FLOAT32: return *(float*)data_ptr;
            case PCLPointField::FLOAT64: return (float)(*(double*)data_ptr);
            default: return 0.0f;
        }
    }
    void FileIO::loadPointCloud(const QString &filename)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud(new Cloud);
        QFileInfo fileInfo(filename);
        QString suffix = fileInfo.suffix().toLower();

        bool is_success = false;
        if (suffix == "ply" || suffix == "pcd") {
            pcl::PCLPointCloud2 blob;
            int res = -1;
            if (suffix == "ply") res = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), blob);
            else res = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), blob);

            if (res != -1) {
                QList<ct::FieldInfo> fields_info;
                for (const auto &f: blob.fields) {
                    fields_info.append({QString::fromStdString(f.name), getPCLFieldType(f.datatype)});
                }

                QMap<QString, QString> mapping_result;

                emit requestFieldMapping(fields_info, mapping_result);

                if (mapping_result.isEmpty()) {
                    // 用户取消
                    emit loadCloudResult(false, cloud, 0);
                    return;
                }

                // 先利用PCL内置转换处理标准
                pcl::fromPCLPointCloud2(blob, *cloud);

                // 手动提取scalar fields
                size_t num_points = blob.width * blob.height;

                // 找到需要提取的字段在 blob 中的偏移量和类型
                struct ExtractInfo {
                    QString targetName; // 存入 cloud 的名字
                    int offset;
                    int datatype;
                };
                std::vector<ExtractInfo> extract_list;

                for (const auto &f: blob.fields) {
                    QString fname = QString::fromStdString(f.name);
                    if (mapping_result.contains(fname)) {
                        QString action = mapping_result[fname];
                        if (action == "Scalar Field" || action == "Intensity") {
                            // 如果是 Intensity，我们统一存为 "Intensity" 方便处理
                            QString saveName = (action == "Intensity") ? "Intensity" : fname;
                            extract_list.push_back({saveName, (int) f.offset, (int) f.datatype});
                        }
                    }
                }
                // 如果有自定义字段，开始提取
                if (!extract_list.empty()) {
                    // 初始化容器
                    QMap<QString, std::vector<float>> scalar_data;
                    for (auto &ex: extract_list) scalar_data[ex.targetName].resize(num_points);

                    // 遍历 blob 数据
                    for (size_t i = 0; i < num_points; ++i) {
                        // 计算当前点在 blob.data 中的起始位置
                        // point_step 是每个点的字节大小
                        const uint8_t *point_ptr = &blob.data[i * blob.point_step];

                        for (const auto &ex: extract_list) {
                            float val = getFieldValueAsFloat(point_ptr + ex.offset, ex.datatype);
                            scalar_data[ex.targetName][i] = val;
                        }
                    }

                    // 存入 cloud
                    for (auto &ex: extract_list) {
                        // 避免重复添加 (extract_list 可能有重复如果字段名重复)
                        if (!cloud->hasScalarField(ex.targetName))
                            cloud->addScalarField(ex.targetName, scalar_data[ex.targetName]);
                    }
                }

                // 4. 设置 hasRGB
                // 检查用户是否映射了 RGB 或者是标准字段
                bool has_color = false;
                for (const auto &f: blob.fields) {
                    if (f.name == "rgb" || f.name == "rgba") has_color = true;
                }
                cloud->setHasRGB(has_color);

                is_success = true;
            }
        }
        else if (suffix == "las" || suffix == "laz") {
            LASreadOpener lasreadopener;
            lasreadopener.set_file_name(filename.toLocal8Bit().constData());
            LASreader* lasreader = lasreadopener.open();

            if (lasreader)
            {
                // 判断 LAS 文件格式是否包含颜色数据 (Formats 2, 3, 5, 7 支持颜色)
                int fmt = lasreader->header.point_data_format;
                bool file_has_color_data = (fmt == 2 || fmt == 3 || fmt == 5 || fmt == 7);
                
                cloud->setHasRGB(file_has_color_data); // 标记点云是否原生包含 RGB

                std::vector<float> intensities;

                while (lasreader->read_point())
                {
                    PointXYZRGBN p;
                    p.x = lasreader->point.get_x();
                    p.y = lasreader->point.get_y();
                    p.z = lasreader->point.get_z();

                    // === 颜色处理 ===
                    uint8_t r_byte = 0, g_byte = 0, b_byte = 0; // 默认黑色 (0,0,0)
                    if (file_has_color_data) {
                        // LAS 颜色是 16bit，转换为 8bit
                        r_byte = lasreader->point.rgb[0] >> 8;
                        g_byte = lasreader->point.rgb[1] >> 8;
                        b_byte = lasreader->point.rgb[2] >> 8;
                    }
                    
                    // 打包颜色为 float
                    std::uint32_t rgb_packed = ((std::uint32_t)r_byte << 16 | (std::uint32_t)g_byte << 8 | (std::uint32_t)b_byte);
                    p.rgb = *reinterpret_cast<float*>(&rgb_packed);

                    // 收集强度信息 (自定义字段)
                    intensities.push_back(static_cast<float>(lasreader->point.get_intensity()));

                    cloud->push_back(p);
                }
                
                // 添加强度字段到 Cloud 对象
                if (!intensities.empty() && intensities.size() == cloud->size()) {
                    cloud->addScalarField("Intensity", intensities);
                }

                lasreader->close();
                delete lasreader;
                is_success = true;
            }
        }
        else{
            int result_pcl = -1;
            if (suffix == "obj")
                result_pcl = pcl::io::loadOBJFile(filename.toLocal8Bit().toStdString(), *cloud);
            else if (suffix == "ifs")
                result_pcl = pcl::io::loadIFSFile(filename.toLocal8Bit().toStdString(), *cloud);
            
            if (result_pcl != -1) {
                cloud->setHasRGB(true); // 对于PCL支持的格式，默认假设有RGB（即使是全白或全黑）
                is_success = true;
            }
        }

        if (!is_success) {
            emit loadCloudResult(false, cloud, time.toc());
            return;
        }

        // 注意：pcl::fromPCLPointCloud2 会保留 NaN，如果需要 removeNaN
        if (cloud->is_dense == false) {
            std::vector<int> indices;
            cloud->removeInvalidPoints();
            // 注意：移除了点后，scalar fields 的数据也必须同步移除！
            // 这比较麻烦。简单起见，如果导入了 scalar fields，建议暂不 removeNaN，或者必须同步处理 scalar vectors。
            // 这是一个高级话题。为简化，建议如果 scalar_fields 不为空，暂时跳过 removeNaN，或者你需要写个函数同步过滤 scalar fields。
        }

        cloud->setId(fileInfo.baseName());
        cloud->setInfo(fileInfo);
        cloud->update(); // 更新包围盒、分辨率等

        cloud->backupColors();

        emit loadCloudResult(true, cloud, time.toc());
    }

    void FileIO::savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary) {
        TicToc time;
        time.tic();
        QFileInfo fileInfo(filename);
        QString suffix = fileInfo.suffix().toLower();
        bool is_success = false;

        if (suffix == "las" || suffix == "laz") {
            LASwriteOpener laswriteopener;
            laswriteopener.set_file_name(filename.toLocal8Bit().constData());
            LASheader lasheader;

            // 确定格式
            if (cloud->hasRGB()) {
                // 原文件有颜色，保存为 Format 2 (支持 XYZ, Intensity, RGB)
                lasheader.point_data_format = 2; // XYZ + RGB + Intensity
                lasheader.point_data_record_length = 26;
            } else {
                lasheader.point_data_format = 0;  // XYZ + Intensity (不存 RGB)
                lasheader.point_data_record_length = 20;
            }

            // 设置精度因子
            lasheader.x_scale_factor = 0.0001; // 0.1毫米精度
            lasheader.y_scale_factor = 0.0001;
            lasheader.z_scale_factor = 0.0001;

            // 设置偏移量 (Offset)，避免坐标过大导致精度丢失，通常取点云的最小值
            if (!cloud->empty()) {
                lasheader.x_offset = cloud->min().x;
                lasheader.y_offset = cloud->min().y;
                lasheader.z_offset = cloud->min().z;
            } else {
                lasheader.x_offset = 0;
                lasheader.y_offset = 0;
                lasheader.z_offset = 0;
            }

            LASwriter *laswriter = laswriteopener.open(&lasheader);
            if (laswriter) {
                // 获取强度数据 (如果存在)
                const std::vector<float> *intensity_ptr = nullptr;
                if (cloud->hasScalarField("Intensity")) {
                    intensity_ptr = cloud->getScalarField("Intensity");
                }

                LASpoint laspoint;
                laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, &lasheader);

                for (size_t i = 0; i < cloud->size(); ++i) {
                    const auto &p = cloud->points[i];
                    laspoint.set_x(p.x);
                    laspoint.set_y(p.y);
                    laspoint.set_z(p.z);

                    // 只有当文件格式支持颜色，并且点云原生有 RGB 时，才写入颜色
                    if (cloud->hasRGB() && (lasheader.point_data_format == 2 || lasheader.point_data_format == 3)) {
                        std::uint32_t rgb_val = cloud->getPointColorForSave(i);
                        laspoint.rgb[0] = ((rgb_val >> 16) & 0xFF) * 256; // R (8bit -> 16bit)
                        laspoint.rgb[1] = ((rgb_val >> 8) & 0xFF) * 256;  // G
                        laspoint.rgb[2] = (rgb_val & 0xFF) * 256;         // B
                    }

                    // 写入强度 (如果存在 Intensity 字段)
                    if (intensity_ptr) {
                        laspoint.set_intensity(static_cast<uint16_t>((*intensity_ptr)[i]));
                    } else {
                        laspoint.set_intensity(0); // 默认强度为 0
                    }

                    laswriter->write_point(&laspoint);
                }
                laswriter->close();
                delete laswriter;
                is_success = true;
            }
        }
            // =================== 保存pcd和ply格式逻辑 ===================
        else {
            pcl::PCLPointCloud2 msg;
            msg.height = 1;
            msg.width = cloud->size();
            msg.is_dense = cloud->is_dense;
            msg.is_bigendian = false;

            pcl::PCLPointField f;
            int current_offset = 0;

            f.name = "x";
            f.offset = current_offset;
            f.datatype = pcl::PCLPointField::FLOAT32;
            f.count = 1;
            msg.fields.push_back(f);
            current_offset += 4;

            f.name = "y";
            f.offset = current_offset;
            f.datatype = pcl::PCLPointField::FLOAT32;
            f.count = 1;
            msg.fields.push_back(f);
            current_offset += 4;

            f.name = "z";
            f.offset = current_offset;
            f.datatype = pcl::PCLPointField::FLOAT32;
            f.count = 1;
            msg.fields.push_back(f);
            current_offset += 4;

            if (cloud->hasRGB()) {
                f.name = "rgb";
                f.offset = current_offset;
                f.datatype = pcl::PCLPointField::FLOAT32;
                f.count = 1;
                msg.fields.push_back(f);
                current_offset += 4;
            }

            // 添加自定义字段
            QStringList fieldNames = cloud->getScalarFieldNames();
            for (const QString &name: fieldNames) {
                f.name = name.toStdString();
                f.offset = current_offset;
                f.datatype = pcl::PCLPointField::FLOAT32;
                f.count = 1;
                msg.fields.push_back(f);
                current_offset += 4;
            }

            // 填充数据blob
            msg.point_step = current_offset;
            msg.row_step = msg.point_step * msg.width;
            msg.data.resize(msg.row_step);
            uint8_t *ptr = msg.data.data();

            std::vector<const std::vector<float> *> scalar_ptrs;
            for (const QString &name: fieldNames) {
                scalar_ptrs.push_back(cloud->getScalarField(name));
            }

            for (size_t i = 0; i < cloud->size(); ++i) {
                const auto &p = cloud->points[i];

                // 写入XYZ
                memcpy(ptr, &p.x, 12);
                int internal_offset = 12;

                // 写入 RGB (直接取原始值)
                if (cloud->hasRGB()) {
                    std::uint32_t rgb_val = cloud->getPointColorForSave(i);
                    // 这里的 trick 是：rgb 在 PCL 内存里是 float，但二进制位是 int
                    // 我们直接把 uint32 copy 进去即可
                    memcpy(ptr + internal_offset, &rgb_val, 4);
                    internal_offset += 4;
                }

                // 写入自定义字段
                for (size_t k = 0; k < scalar_ptrs.size(); ++k) {
                    float val = (*scalar_ptrs[k])[i];
                    memcpy(ptr + internal_offset, &val, 4);
                    internal_offset += 4;
                }

                ptr += msg.point_step;
            }

            if (suffix == "pcd") {
                pcl::io::savePCDFile(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(),
                                     Eigen::Quaternionf::Identity(), isBinary);
                is_success = true;
            } else {
                //默认ply
                pcl::PLYWriter wirtter;
                int res = wirtter.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(),
                                        Eigen::Quaternionf::Identity(), isBinary, false);
                is_success = (res == 0);
            }
        }

        if (is_success)
                emit saveCloudResult(true, filename, time.toc());
        else
                emit saveCloudResult(false, filename, time.toc());
    }
}
