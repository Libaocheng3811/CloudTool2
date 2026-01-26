#include "fileio.h"

#include "pcl/filters/filter.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl/io/ifs_io.h"
#include "pcl/io/obj_io.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/common.h>

#include "lasreader.hpp"
#include "laswriter.hpp"

#include <fstream>
#include <string>
#include <iomanip> // std::setprecision

namespace ct
{
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

    // 辅助函数：安全地从指针读取值
    template<typename T>
    T safeRead(const uint8_t* base_ptr, size_t offset, size_t max_size) {
        size_t required = offset + sizeof(T);
        if (required > max_size || offset >= max_size) {
            // 超出边界，返回默认值
            return T();
        }
        return *reinterpret_cast<const T*>(base_ptr + offset);
    }

    // 辅助函数：从 Blob 中提取 float 值（带边界检查）
    float getFieldValueAsFloat(const uint8_t* data_ptr, int datatype, size_t max_size) {
        using namespace pcl;
        switch(datatype) {
            case PCLPointField::INT8:
                if (max_size >= 1) return (float)(*(int8_t*)data_ptr);
                break;
            case PCLPointField::UINT8:
                if (max_size >= 1) return (float)(*(uint8_t*)data_ptr);
                break;
            case PCLPointField::INT16:
                if (max_size >= 2) return (float)(*(int16_t*)data_ptr);
                break;
            case PCLPointField::UINT16:
                if (max_size >= 2) return (float)(*(uint16_t*)data_ptr);
                break;
            case PCLPointField::INT32:
                if (max_size >= 4) return (float)(*(int32_t*)data_ptr);
                break;
            case PCLPointField::UINT32:
                if (max_size >= 4) return (float)(*(uint32_t*)data_ptr);
                break;
            case PCLPointField::FLOAT32:
                if (max_size >= 4) return *(float*)data_ptr;
                break;
            case PCLPointField::FLOAT64:
                if (max_size >= 8) return (float)(*(double*)data_ptr);
                break;
            default:
                break;
        }
        return 0.0f;  // 默认值
    }

    void FileIO::loadPointCloud(const QString &filename) {
        TicToc time;
        time.tic();
        m_is_canceled = false;
        emit progress(0);

        Cloud::Ptr cloud(new Cloud);
        QFileInfo fileInfo(filename);
        QString suffix = fileInfo.suffix().toLower();
        bool is_success = false;

        //根据后缀分发处理
        if (suffix == "las" || suffix == "laz"){
            is_success = loadLAS(filename, cloud);
        }
        else if (suffix == "ply" || suffix == "pcd"){
            is_success = loadPLY_PCD(filename, cloud);
        }
        else if (suffix == "txt" || suffix == "xyz" || suffix == "asc"){
            is_success = loadTXT(filename, cloud);
        }
        else{
            is_success = loadGeneralPCL(filename, cloud);
        }

        // 失败处理
        if (!is_success || m_is_canceled){
            emit loadCloudResult(false, cloud, time.toc());
            return;
        }

        emit progress(90);

        cloud->setId(fileInfo.baseName());
        cloud->setInfo(fileInfo);
        cloud->update(); //更新包围盒，统计信息

        emit progress(100);
        emit loadCloudResult(true, cloud, time.toc());
    }

    void FileIO::savePointCloud(const Cloud::Ptr &cloud, const QString &filename, bool isBinary) {
        TicToc time;
        time.tic();
        QFileInfo fileInfo(filename);
        QString suffix = fileInfo.suffix().toLower();
        bool is_success = false;

        // 恢复原始数据
        cloud->restoreColors();

        //检查是否存在偏移
        Eigen::Vector3d shift = cloud->getGlobalShift();
        bool has_shift = (shift != Eigen::Vector3d::Zero());

        if (suffix == "las" || suffix == "laz"){
            is_success = saveLAS(cloud, filename);
        }
        else if (suffix == "txt" || suffix == "xyz" || suffix == "csv"){
            is_success = saveTXT(cloud, filename);
        }
        else {
            is_success = savePCL(cloud, filename, isBinary);
        }

        if (is_success) emit saveCloudResult(true, filename, time.toc());
        else emit saveCloudResult(false, filename, time.toc());
    }

    bool FileIO::loadLAS(const QString &filename, Cloud::Ptr &cloud) {
        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(filename.toLocal8Bit().constData());
        LASreader* lasreader = lasreadopener.open();
        if (!lasreader) return false;

        size_t total_points = lasreader->npoints;
        if (total_points == 0){
            lasreader->close();
            delete lasreader;
            return false;
        }

        cloud->m_xyz->reserve(total_points);

        int fmt = lasreader->header.point_data_format;
        bool has_color = (fmt == 2 || fmt == 3 || fmt == 5 || fmt == 7);
        if (has_color) cloud->enableColors();

        // 强度字段
        bool has_valid_intensity = false;
        std::vector<float> intensities;
        intensities.reserve(total_points);

        size_t lod_step = (total_points > PREVIEW_LIMIT) ? (total_points / PREVIEW_LIMIT) : 1;
        // 只有当需要降采样时才创建预览云，否则直接用全量（Cloud::getPreviewCloud 会处理）
        if (lod_step > 1) {
            cloud->m_preview = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
            cloud->m_preview->reserve(PREVIEW_LIMIT + 200);
        }

        // 读取第一个点以确定 Global Shift
        if (!lasreader->read_point()) {
            lasreader->close();
            delete lasreader;
            return false;
        }

        double raw_x = lasreader->point.get_x();
        double raw_y = lasreader->point.get_y();
        double raw_z = lasreader->point.get_z();

        const double THRESHOLD_XY = 10000.0;
        bool is_large = std::abs(raw_x) > THRESHOLD_XY || std::abs(raw_y) > THRESHOLD_XY;
        Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
        if (is_large) {
            double sx = -std::floor(raw_x / 1000.0) * 1000.0;
            double sy = -std::floor(raw_y / 1000.0) * 1000.0;
            double sz = 0.0; // 高程通常不需要偏移，或者按需

            suggested_shift = Eigen::Vector3d(sx, sy, sz);
            // 请求用户确认 (阻塞信号)
            bool skipped = false;
            emit requestGlobalShift(Eigen::Vector3d(raw_x, raw_y, raw_z), suggested_shift, skipped);

            if (!skipped) {
                cloud->setGlobalShift(-suggested_shift); // Cloud 存储的是 Origin (-Shift)
            }
        }

        float shift_x = static_cast<float>(suggested_shift.x());
        float shift_y = static_cast<float>(suggested_shift.y());
        float shift_z = static_cast<float>(suggested_shift.z());

        // 处理第一个点
        {
            float x = static_cast<float>(raw_x) + shift_x;
            float y = static_cast<float>(raw_y) + shift_y;
            float z = static_cast<float>(raw_z) + shift_z;

            cloud->m_xyz->push_back(pcl::PointXYZ(x, y, z));

            uint8_t r = 0, g = 0, b = 0;
            if (has_color) {
                // LAS 颜色通常是 16位 (0-65535)，需要压缩到 8位
                r = lasreader->point.rgb[0] >> 8;
                g = lasreader->point.rgb[1] >> 8;
                b = lasreader->point.rgb[2] >> 8;
                cloud->m_colors->push_back({r, g, b});
            }

            // 读取第一个点的强度，判断是否有效
            float intensity_val = static_cast<float>(lasreader->point.get_intensity());
            intensities.push_back(intensity_val);
            if (intensity_val > 0.0f) has_valid_intensity = true;

            // [LOD 写入]
            if (lod_step > 1) {
                pcl::PointXYZRGB p;
                p.x = x; p.y = y; p.z = z;
                p.r = r; p.g = g; p.b = b;
                if (!has_color) { p.r = 255; p.g = 255; p.b = 255; }
                cloud->m_preview->push_back(p);
            }
        }

        size_t idx = 1;
        int progress_interval = (total_points > 100) ? (total_points / 100) : 1;

        while (lasreader->read_point()) {
            if (m_is_canceled) {
                lasreader->close();
                delete lasreader;
                return false;
            }

            // 坐标
            float x = static_cast<float>(lasreader->point.get_x()) + shift_x;
            float y = static_cast<float>(lasreader->point.get_y()) + shift_y;
            float z = static_cast<float>(lasreader->point.get_z()) + shift_z;

            cloud->m_xyz->push_back(pcl::PointXYZ(x, y, z));

            // 颜色
            uint8_t r = 0, g = 0, b = 0;
            if (has_color) {
                r = lasreader->point.rgb[0] >> 8;
                g = lasreader->point.rgb[1] >> 8;
                b = lasreader->point.rgb[2] >> 8;
                cloud->m_colors->push_back({r, g, b});
            }

            // 读取第一个点的强度，判断是否有效
            float intensity_val = static_cast<float>(lasreader->point.get_intensity());
            intensities.push_back(intensity_val);
            if (!has_valid_intensity && intensity_val > 0.0f) has_valid_intensity = true;

            // [LOD 实时生成]
            // 如果需要 LOD 且命中步长，加入预览云
            // todo 这里的预览点云是怎么选取的，选点查看信息时，为什么不直接使用预览点云的点信息，而是要转换会原始点云索引呢？
            if (lod_step > 1 && idx % lod_step == 0) {
                pcl::PointXYZRGB p;
                p.x = x; p.y = y; p.z = z;
                if (has_color) {
                    p.r = r; p.g = g; p.b = b;
                } else {
                    p.r = 255; p.g = 255; p.b = 255;
                }
                cloud->m_preview->push_back(p);
            }

            // 更新进度条 (5% ~ 90%)
            idx++;
            if (idx % progress_interval == 0) {
                int p = 5 + (int)(idx * 85 / total_points);
                emit progress(p);
            }
        }

        // 只有强度字段不为空时才添加
        if (has_valid_intensity && !intensities.empty()) {
            cloud->addScalarField("Intensity", intensities);
        }

        cloud->setHasRGB(has_color);

        lasreader->close();
        delete lasreader;
        return true;
    }

    bool FileIO::loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud) {
        pcl::PCLPointCloud2 blob;
        int res = -1;

        // 先初步读取ply信息到blob中
        if (filename.endsWith(".ply", Qt::CaseInsensitive))
            res = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), blob);
        else
            res = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), blob); // todo 这步十分耗时

        if (res == -1) return false;

        // 准备字段信息
        QList<ct::FieldInfo> fields_info;
        for (const auto &f: blob.fields) {
            fields_info.append({QString::fromStdString(f.name), getPCLFieldType(f.datatype)});
        }

        // 请求 UI, mapping_result是用户选择的映射结果，Key是原字段名，Value是映射后的字段名
        QMap<QString, QString> mapping_result;
        emit requestFieldMapping(fields_info, mapping_result);
        if (mapping_result.isEmpty()) return false; // 用户取消

        size_t total_points = blob.width * blob.height;
        if (total_points == 0) return false;

        // 获取 blob 数据大小，用于后续边界检查
        size_t blob_data_size = blob.data.size();

        cloud->m_xyz->resize(total_points);

        int x_off = -1, y_off = -1, z_off = -1;
        for (const auto& f : blob.fields){
            if (f.name == "x") x_off = f.offset;
            if (f.name == "y") y_off = f.offset;
            if (f.name == "z") z_off = f.offset;
        }
        if (x_off < 0 || y_off < 0 || z_off < 0) return false;

        // 处理Global Shift（带边界检查）
        Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
        const uint8_t* ptr0 = blob.data.data();
        int max_offset = std::max({x_off, y_off, z_off}) + 4;

        // 检查是否有足够的数据读取第一个点的坐标
        if (blob_data_size >= max_offset) {
            float x0 = *reinterpret_cast<const float*>(ptr0 + x_off);
            float y0 = *reinterpret_cast<const float*>(ptr0 + y_off);
            float z0 = *reinterpret_cast<const float*>(ptr0 + z_off);

            const double THRESHOLD_XY = 10000.0;
            if (std::abs(x0) > THRESHOLD_XY || std::abs(y0) > THRESHOLD_XY) {
                double sx = -std::floor(x0 / 1000.0) * 1000.0;
                double sy = -std::floor(y0 / 1000.0) * 1000.0;
                double sz = 0.0;

                suggested_shift = Eigen::Vector3d(sx, sy, sz);
                bool skipped = false;
                emit requestGlobalShift(Eigen::Vector3d(x0, y0, z0), suggested_shift, skipped);
                if (!skipped) {
                    cloud->setGlobalShift(-suggested_shift);
                }
            }
        }
        emit progress(5);

        // 准备解析
        struct ExtractInfo {
            enum Type { Scalar, ColorPacked, ColorR, ColorG, ColorB }; // 支持分开的颜色分量
            Type type;
            QString saveName;
            int offset;
            int datatype;
        };
        std::vector<ExtractInfo> tasks;
        bool has_color = false;
        bool has_normal = false;
        int nx_off = -1, ny_off = -1, nz_off = -1;  // 法线分量的偏移量
        int nx_datatype = 0, ny_datatype = 0, nz_datatype = 0;  // 法线分量的数据类型
        int r_off = -1, g_off = -1, b_off = -1;  // 颜色分量的偏移量
        int r_datatype = 0, g_datatype = 0, b_datatype = 0;  // 颜色分量的数据类型

        // 遍历映射表，确定要提取哪些字段
        bool use_packed_color = false;  // 标记是否使用打包颜色
        for (const auto& f : blob.fields) {
            QString fname = QString::fromStdString(f.name);
            if (!mapping_result.contains(fname)) continue;

            QString action = mapping_result[fname];
            if (action == "Scalar Field" || action == "Intensity") {
                QString saveName = (action == "Intensity") ? "Intensity" : fname;
                tasks.push_back({ExtractInfo::Scalar, saveName, (int)f.offset, f.datatype});
            }
            else if (action.contains("Color") && !action.contains("Red") && !action.contains("Green") && !action.contains("Blue")) {
                // 打包的颜色字段 (rgb 或 rgba)
                has_color = true;
                use_packed_color = true;
                tasks.push_back({ExtractInfo::ColorPacked, "", (int)f.offset, f.datatype});
            }
            else if (action.contains("Red") || action == "r"){
                has_color = true;
                r_off = f.offset;
                r_datatype = f.datatype;
            }
            else if (action.contains("Green") || action == "g"){
                has_color = true;
                g_off = f.offset;
                g_datatype = f.datatype;
            }
            else if (action.contains("Blue") || action == "b"){
                has_color = true;
                b_off = f.offset;
                b_datatype = f.datatype;
            }
            else if (action.contains("Normal X")){
                has_normal = true;
                nx_off = f.offset;
                nx_datatype = f.datatype;
            }
            else if (action.contains("Normal Y")){
                has_normal = true;
                ny_off = f.offset;
                ny_datatype = f.datatype;
            }
            else if (action.contains("Normal Z")){
                has_normal = true;
                nz_off = f.offset;
                nz_datatype = f.datatype;
            }
        }

        // 优先使用分开的颜色分量，如果没有才使用打包颜色
        if (has_color && !use_packed_color && (r_off < 0 || g_off < 0 || b_off < 0)) {
            // 如果用户选择了颜色但没有完整的 RGB 分量，禁用颜色
            has_color = false;
        }

        if (has_color) cloud->enableColors();
        // 如果检测到有法线字段，预分配法线容器
        if (has_normal) cloud->enableNormals();

        // 分配标量场空间
        QMap<QString, std::vector<float>> scalar_data;
        for(const auto& t : tasks) {
            if (t.type == ExtractInfo::Scalar) {
                scalar_data[t.saveName].resize(total_points);
            }
        }

        // 5. 并行提取数据
        // 为了线程安全访问 scalar_data，我们构建一个指针查找表
        std::vector<float*> task_scalar_ptrs;
        task_scalar_ptrs.reserve(tasks.size());

        for (const auto& t : tasks) {
            if (t.type == ExtractInfo::Scalar) {
                task_scalar_ptrs.push_back(scalar_data[t.saveName].data());
            } else {
                task_scalar_ptrs.push_back(nullptr); // 占位
            }
        }

        float shift_x = (float)suggested_shift.x();
        float shift_y = (float)suggested_shift.y();
        float shift_z = (float)suggested_shift.z();

        int step = blob.point_step;
        const uint8_t* raw_data = blob.data.data();

        // 验证数据完整性并调整点数
        size_t max_valid_points = blob_data_size / step;
        if (max_valid_points < total_points) {
            // 数据不完整，调整点云大小
            total_points = max_valid_points;
            cloud->m_xyz->resize(total_points);
            // 重新启用颜色和法线，确保大小正确
            if (has_color) {
                cloud->enableColors();
                cloud->m_colors->resize(total_points);
            }
            if (has_normal) {
                cloud->enableNormals();
                cloud->m_normals->resize(total_points);
            }
        }

        // 开启 OpenMP 加速
        // 将 blob_data_size 捕获为局部变量，在并行区域中使用
        const size_t data_size = blob_data_size;
        // 计算最大字段偏移量，用于边界检查
        const int max_field_offset = std::max({x_off, y_off, z_off, r_off, g_off, b_off,
                                                nx_off, ny_off, nz_off, 0});
        const size_t max_bytes_needed = static_cast<size_t>(max_field_offset) + sizeof(float);

#pragma omp parallel for
        for (long long i = 0; i < (long long)total_points; ++i) {
            if (m_is_canceled) continue;

            const uint8_t* pt_ptr = raw_data + (size_t)i * step;

            // 计算从 pt_ptr 到 blob.data 末尾的剩余字节数
            size_t remaining_from_pt_ptr = data_size - static_cast<size_t>(pt_ptr - raw_data);

            // 检查是否超出了数据边界
            if (remaining_from_pt_ptr < max_bytes_needed) {
                // 数据不完整，跳过这个点或使用默认值
                cloud->m_xyz->points[i].x = shift_x;
                cloud->m_xyz->points[i].y = shift_y;
                cloud->m_xyz->points[i].z = shift_z;
                continue;
            }

            // 最大安全偏移量 = 当前点到数据末尾的距离
            size_t max_offset = remaining_from_pt_ptr;

            // 提取 XYZ（带边界检查）
            float x = 0, y = 0, z = 0;
            if (x_off >= 0 && (size_t)x_off + sizeof(float) <= max_offset) memcpy(&x, pt_ptr + x_off, sizeof(float));
            if (y_off >= 0 && (size_t)y_off + sizeof(float) <= max_offset) memcpy(&y, pt_ptr + y_off, sizeof(float));
            if (z_off >= 0 && (size_t)z_off + sizeof(float) <= max_offset) memcpy(&z, pt_ptr + z_off, sizeof(float));

            cloud->m_xyz->points[i].x = x + shift_x;
            cloud->m_xyz->points[i].y = y + shift_y;
            cloud->m_xyz->points[i].z = z + shift_z;

            // 提取其他属性
            for (size_t k = 0; k < tasks.size(); ++k) {
                const auto& t = tasks[k];

                // 提取标量场
                if (t.type == ExtractInfo::Scalar) {
                    size_t remaining_size = max_offset;
                    if (t.offset >= 0 && t.offset < max_offset) {
                        remaining_size = max_offset - t.offset;
                    }
                    float val = getFieldValueAsFloat(pt_ptr + t.offset, t.datatype, remaining_size);
                    task_scalar_ptrs[k][i] = val;
                }
                else if (t.type == ExtractInfo::ColorPacked) { // 提取打包的颜色
                    if (t.offset >= 0 && (size_t)t.offset + 4 <= max_offset) {
                        uint32_t rgb_packed = safeRead<uint32_t>(pt_ptr, t.offset, max_offset);
                        (*cloud->m_colors)[i].r = (rgb_packed >> 16) & 0xFF;
                        (*cloud->m_colors)[i].g = (rgb_packed >> 8) & 0xFF;
                        (*cloud->m_colors)[i].b = rgb_packed & 0xFF;
                    } else {
                        // 默认白色
                        (*cloud->m_colors)[i].r = 255;
                        (*cloud->m_colors)[i].g = 255;
                        (*cloud->m_colors)[i].b = 255;
                    }
                }
            }

            // 提取分开的颜色分量（如果存在）
            if (has_color && r_off >= 0 && g_off >= 0 && b_off >= 0) {
                uint8_t r = 255, g = 255, b = 255;

                if (r_off >= 0 && r_off < max_offset) {
                    r = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + r_off, r_datatype, max_offset - r_off));
                }
                if (g_off >= 0 && g_off < max_offset) {
                    g = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + g_off, g_datatype, max_offset - g_off));
                }
                if (b_off >= 0 && b_off < max_offset) {
                    b = static_cast<uint8_t>(getFieldValueAsFloat(pt_ptr + b_off, b_datatype, max_offset - b_off));
                }

                (*cloud->m_colors)[i].r = r;
                (*cloud->m_colors)[i].g = g;
                (*cloud->m_colors)[i].b = b;
            }

            // 提取法线（如果存在）
            if (has_normal && nx_off >= 0 && ny_off >= 0 && nz_off >= 0) {
                float nx = 0, ny = 0, nz = 0;

                if (nx_off >= 0 && (size_t)nx_off + 4 <= max_offset) {
                    nx = getFieldValueAsFloat(pt_ptr + nx_off, nx_datatype, max_offset - nx_off);
                }
                if (ny_off >= 0 && (size_t)ny_off + 4 <= max_offset) {
                    ny = getFieldValueAsFloat(pt_ptr + ny_off, ny_datatype, max_offset - ny_off);
                }
                if (nz_off >= 0 && (size_t)nz_off + 4 <= max_offset) {
                    nz = getFieldValueAsFloat(pt_ptr + nz_off, nz_datatype, max_offset - nz_off);
                }

                ct::CompressedNormal cn;
                cn.set(Eigen::Vector3f(nx, ny, nz));
                (*cloud->m_normals)[i] = cn;
            }
        }

        for (auto it = scalar_data.begin(); it != scalar_data.end(); ++it) {
            cloud->addScalarField(it.key(), it.value());
        }

        cloud->setHasRGB(has_color);
        cloud->setHasNormals(has_normal);

        emit progress(80);

        if (total_points > PREVIEW_LIMIT){
            cloud->generatePreview();
        }

        cloud->m_xyz->is_dense = blob.is_dense;
        return true;
    }

    bool FileIO::loadTXT(const QString &filename, Cloud::Ptr &cloud) {
        std::ifstream file(filename.toLocal8Bit().constData());
        if (!file.is_open()) return false;

        //获取文件大小
        file.seekg(0, std::ios::end);
        long long file_size = file.tellg();
        file.seekg(0, std::ios::beg);

        // 预读
        QStringList preview_lines;
        std::string line;
        int preview_count = 0;
        while (std::getline(file, line) && preview_count < 50) {
            if (!line.empty()) {
                preview_lines << QString::fromStdString(line);
                preview_count++;
            }
        }
        file.clear();
        file.seekg(0);

        // 交互配置
        ct::TxtImportParams params;
        emit requestTxtImportSetup(preview_lines, params);

        // 检查配置
        bool has_x = false, has_y = false, has_z = false;
        for(auto val : params.col_map) {
            if(val == "x") has_x = true;
            if(val == "y") has_y = true;
            if(val == "z") has_z = true;
        }
        if (params.col_map.isEmpty() || !has_x || !has_y || !has_z) return false;

        // 解析
        for(int i=0; i<params.skip_lines; ++i) std::getline(file, line);

        std::streampos  data_start_pos = file.tellg();

        if (std::getline(file, line)){
            // 解析第一行
            QString qline = QString::fromStdString(line);
            QStringList parts;
            if (params.separator == ' ') parts = qline.simplified().split(' ', QString::SkipEmptyParts);
            else parts = qline.split(params.separator);

            double first_x = 0, first_y = 0, first_z = 0;
            for (auto it = params.col_map.begin(); it != params.col_map.end(); ++it){
                if (it.key() < parts.size()){
                    if (it.value() == "x") first_x = parts[it.key()].toDouble();
                    if (it.value() == "y") first_y = parts[it.key()].toDouble();
                    if (it.value() == "z") first_z = parts[it.key()].toDouble();
                }
            }

            const double THRESHOLD_XY = 10000.0;
            if(std::abs(first_x) > THRESHOLD_XY || std::abs(first_y) > THRESHOLD_XY){
                double shiftX = -std::floor(first_x / 1000.0) * 1000.0;
                double shiftY = -std::floor(first_y / 1000.0) * 1000.0;
                double shiftZ = 0.0;

                Eigen::Vector3d first_point(first_x, first_y, first_z);
                Eigen::Vector3d suggested_shift(shiftX, shiftY, shiftZ);
                bool skipped = false;

                emit requestGlobalShift(first_point, suggested_shift, skipped);

                if (!skipped) cloud->setGlobalShift(-suggested_shift);
            }
        }
        file.clear();
        file.seekg(data_start_pos);

        emit  progress(5);

        long long read_bytes = 0;
        Eigen::Vector3d shift = -cloud->getGlobalShift();
        bool has_shift = (shift != Eigen::Vector3d::Zero());
        float shift_x = (float)shift.x();
        float shift_y = (float)shift.y();
        float shift_z = (float)shift.z();

        bool has_color = false;
        bool has_normal = false;
        QList<int> scalar_indices;
        QList<QString> scalar_names;

        for (auto it = params.col_map.begin(); it != params.col_map.end(); ++it) {
            QString type = it.value();
            if (type == "r" || type == "g" || type == "b") has_color = true;
            if (type.startsWith("n")) has_normal = true; // nx, ny, nz
            if (type != "x" && type != "y" && type != "z" &&
                type != "r" && type != "g" && type != "b" &&
                !type.startsWith("n") && type != "ignore") {
                scalar_indices.append(it.key());
                scalar_names.append(type);
            }
        }

        // 由于TXT无法预知点数，通过文件大小进行估计，并预分配内存
        size_t estimated_points = file_size / 50;

        cloud->m_xyz->reserve(estimated_points);

        // 注意：不提前调用 enableColors/enableNormals，因为此时 m_xyz->size() = 0
        // enableColors/enableNormals 内部调用 resize(m_xyz->size())，会 resize 到 0
        // 改为按需创建并预分配
        std::unique_ptr<std::vector<RGB>> temp_colors;
        std::unique_ptr<std::vector<CompressedNormal>> temp_normals;
        if (has_color) {
            temp_colors = std::make_unique<std::vector<RGB>>();
            temp_colors->reserve(estimated_points);
        }
        if (has_normal) {
            temp_normals = std::make_unique<std::vector<CompressedNormal>>();
            temp_normals->reserve(estimated_points);
        }

        QMap<QString, std::vector<float>> scalar_data;
        for(const QString& name : scalar_names) {
            scalar_data[name].reserve(estimated_points);
        }

        if (cloud->m_preview == nullptr) {
            cloud->m_preview = std::make_shared<pcl::PointCloud<PointXYZRGB>>();
            cloud->m_preview->reserve(PREVIEW_LIMIT);
        }

        int lod_step = (estimated_points > PREVIEW_LIMIT) ? (estimated_points / PREVIEW_LIMIT) : 1;
        if (lod_step < 1) lod_step = 1;

        size_t current_point_idx = 0;

        // 读取循环
        while (std::getline(file, line))
        {
            if (m_is_canceled) return false;

            read_bytes += line.size() + 1;
            if (read_bytes % 1024000 == 0) { // 每 1MB 更新
                int p = 5 + (int)(read_bytes * 85 / file_size);
                if (p > 90) p = 90;
                emit progress(p);
            }

            if (line.empty()) continue;

            QString qline = QString::fromStdString(line);
            QStringList parts;
            if (params.separator == ' ') parts = qline.simplified().split(' ', QString::SkipEmptyParts);
            else parts = qline.split(params.separator);

            // 解析
            float x=0, y=0, z=0;
            uint8_t r=255, g=255, b=255;
            float nx=0, ny=0, nz=0;

            for(auto it = params.col_map.begin(); it != params.col_map.end(); ++it) {
                int idx = it.key();
                if (idx >= parts.size()) continue;

                float val = parts[idx].toFloat();
                QString type = it.value();

                if (type == "x") x = val;
                else if (type == "y") y = val;
                else if (type == "z") z = val;
                else if (type == "r") r = (uint8_t)val;
                else if (type == "g") g = (uint8_t)val;
                else if (type == "b") b = (uint8_t)val;
                else if (type == "nx") nx = val;
                else if (type == "ny") ny = val;
                else if (type == "nz") nz = val;
                else if (type != "ignore") {
                    // 标量场
                    scalar_data[type].push_back(val);
                }
            }

            // 应用 Shift
            if (has_shift) {
                x += shift_x;
                y += shift_y;
                z += shift_z;
            }

            // 存入 SOA
            cloud->m_xyz->push_back(pcl::PointXYZ(x, y, z));
            if (has_color) temp_colors->push_back({r, g, b});
            if (has_normal) {
                CompressedNormal cn;
                cn.set(Eigen::Vector3f(nx, ny, nz));
                temp_normals->push_back(cn);
            }

            // 实时 LOD
            if (lod_step > 1 && current_point_idx % lod_step == 0) {
                if (cloud->m_preview->size() < PREVIEW_LIMIT) {
                    pcl::PointXYZRGB p;
                    p.x=x; p.y=y; p.z=z;
                    if(has_color) { p.r=r; p.g=g; p.b=b; }
                    else { p.r=255; p.g=255; p.b=255; }
                    cloud->m_preview->push_back(p);
                }
            }
            current_point_idx++;
        }

        // 如果点数小于预览点数，则不需要生成预览
        if (cloud->m_xyz->size() <= PREVIEW_LIMIT) {
            // 释放预览云内存
            cloud->m_preview.reset();
            cloud->m_preview = nullptr;
        }
        else {
            // 如果点数巨大，且我们在读取时没有生成预览,重新生成
            if (lod_step == 1 && cloud->m_preview == nullptr) {
                cloud->generatePreview(PREVIEW_LIMIT);
            }
        }

        for(auto it = scalar_data.begin(); it != scalar_data.end(); ++it) {
            cloud->addScalarField(it.key(), it.value());
        }

        // 将临时颜色和法线数据转移到 cloud
        if (temp_colors && !temp_colors->empty()) {
            cloud->m_colors = std::move(temp_colors);
            cloud->m_has_rgb = true;
        }
        if (temp_normals && !temp_normals->empty()) {
            cloud->m_normals = std::move(temp_normals);
            cloud->m_has_normals = true;
        }

        cloud->setHasRGB(has_color);
        cloud->setHasNormals(has_normal);

        cloud->m_xyz->is_dense = true;
        return true;
    }

    bool FileIO::loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud) {
        emit progress(5);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        int res = -1;
        if (filename.endsWith(".obj", Qt::CaseInsensitive))
            res = pcl::io::loadOBJFile(filename.toLocal8Bit().toStdString(), *temp_cloud);
        else if (filename.endsWith(".ifs", Qt::CaseInsensitive))
            res = pcl::io::loadIFSFile(filename.toLocal8Bit().toStdString(), *temp_cloud);

        if (res == -1 || temp_cloud->empty()) return false;

        emit progress(50);

        size_t num_points = temp_cloud->size();
        const auto& p0 = temp_cloud->points[0];
        const double THRESHOLD_XY = 10000.0;

        Eigen::Vector3d suggested_shift = Eigen::Vector3d::Zero();
        if (std::abs(p0.x) > THRESHOLD_XY || std::abs(p0.y) > THRESHOLD_XY) {
            double sx = -std::floor(p0.x / 1000.0) * 1000.0;
            double sy = -std::floor(p0.y / 1000.0) * 1000.0;
            double sz = 0.0;

            suggested_shift = Eigen::Vector3d(sx, sy, sz);

            bool skipped = false;
            emit requestGlobalShift(Eigen::Vector3d(p0.x, p0.y, p0.z), suggested_shift, skipped);

            if (!skipped) {
                cloud->setGlobalShift(-suggested_shift);
            }
        }

        cloud->m_xyz->resize(num_points);
        // 假设 OBJ 都有颜色和法线
        cloud->enableColors();
        cloud->enableNormals();

        float shift_x = (float)suggested_shift.x();
        float shift_y = (float)suggested_shift.y();
        float shift_z = (float)suggested_shift.z();

#pragma omp parallel for
        for (int i = 0; i < (int)num_points; ++i) {
            const auto& src = temp_cloud->points[i];

            // XYZ
            cloud->m_xyz->points[i].x = src.x + shift_x;
            cloud->m_xyz->points[i].y = src.y + shift_y;
            cloud->m_xyz->points[i].z = src.z + shift_z;

            // Color
            (*cloud->m_colors)[i] = {src.r, src.g, src.b};

            // Normal
            (*cloud->m_normals)[i].set(Eigen::Vector3f(src.normal_x, src.normal_y, src.normal_z));
        }
        if (m_is_canceled) return false;
        emit progress(80);

        cloud->setHasRGB(!cloud->m_colors->empty());
        cloud->setHasNormals(!cloud->m_normals->empty());

        if (num_points > PREVIEW_LIMIT){
            cloud->generatePreview();
        }

        cloud->m_xyz->is_dense = temp_cloud->is_dense;
        emit progress(100);

        return true;
    }

    bool FileIO::saveLAS(const Cloud::Ptr &cloud, const QString &filename) {
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

        // Global Shift 还原
        Eigen::Vector3d shift = cloud->getGlobalShift();
        if (!cloud->empty()) {
            lasheader.x_offset = cloud->min().x + shift.x();
            lasheader.y_offset = cloud->min().y + shift.y();
            lasheader.z_offset = cloud->min().z + shift.z();
        }

        LASwriter *laswriter = laswriteopener.open(&lasheader);
        if (!laswriter) return false;

        // 预取 Intensity 字段指针 (如果存在)
        const std::vector<float>* intensity_ptr = nullptr;
        if (cloud->hasScalarField("Intensity")) {
            intensity_ptr = cloud->getScalarField("Intensity");
        }

        LASpoint laspoint;
        laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, &lasheader);

        // 循环写入
        size_t num_points = cloud->size();
        for (size_t i = 0; i < num_points; ++i)
        {
            const auto& p = cloud->m_xyz->points[i];
            laspoint.set_x(p.x + shift.x());
            laspoint.set_y(p.y + shift.y());
            laspoint.set_z(p.z + shift.z());

            if (cloud->hasRGB() && cloud->m_colors && i < cloud->m_colors->size()) {
                const auto& c = (*cloud->m_colors)[i];
                laspoint.rgb[0] = c.r * 256;
                laspoint.rgb[1] = c.g * 256;
                laspoint.rgb[2] = c.b * 256;
            }

            // [Intensity]
            if (intensity_ptr) {
                laspoint.set_intensity(static_cast<uint16_t>((*intensity_ptr)[i]));
            } else {
                laspoint.set_intensity(0);
            }

            laswriter->write_point(&laspoint);
        }

        laswriter->close();
        delete laswriter;
        return true;
    }

    bool FileIO::saveTXT(const Cloud::Ptr &cloud, const QString &filename) {
        // 收集可用字段
        QStringList available_fields;
        available_fields << "x" << "y" << "z";
        if (cloud->hasRGB()) available_fields << "r" << "g" << "b";
        if (cloud->hasNormals()) available_fields << "nx" << "ny" << "nz";

        //添加标量字段
        QStringList scalar_names = cloud->getScalarFieldNames();
        available_fields.append(scalar_names);

        //请求配置
        ct::TxtExportParams params;
        emit requestTxtExportSetup(available_fields, params);
        if (params.selected_fields.isEmpty()) return false; // 用户取消

        //打开文件
        std::ofstream file(filename.toLocal8Bit().constData());
        if (!file.is_open()) return false;

        //设置精度
        file << std::fixed << std::setprecision(params.precision);

        //写入header（可选）
        if (params.has_header) {
            for (int i = 0; i < params.selected_fields.size(); i++){
                file << params.selected_fields[i].toStdString();
                if (i < params.selected_fields.size() - 1) file << params.separator;
            }
            file << "\n";
        }

        //准备数据指针以加速访问
        struct FieldPtr {
            enum Type { XYZ, RGB, Normal, Scalar};
            Type type;
            int sub_index; // 0=x/r, 1=y/g, 2=z/b
            const std::vector<float>* scalar_vec; // 标量字段
        };

        std::vector<FieldPtr> field_ptrs;
        for (const QString& f : params.selected_fields){
            if (f == "x") field_ptrs.push_back({FieldPtr::XYZ, 0, nullptr});
            else if (f == "y") field_ptrs.push_back({FieldPtr::XYZ, 1, nullptr});
            else if (f == "z") field_ptrs.push_back({FieldPtr::XYZ, 2, nullptr});
            else if (f == "r") field_ptrs.push_back({FieldPtr::RGB, 0, nullptr});
            else if (f == "g") field_ptrs.push_back({FieldPtr::RGB, 1, nullptr});
            else if (f == "b") field_ptrs.push_back({FieldPtr::RGB, 2, nullptr});
            else if (f == "nx") field_ptrs.push_back({FieldPtr::Normal, 0, nullptr});
            else if (f == "ny") field_ptrs.push_back({FieldPtr::Normal, 1, nullptr});
            else if (f == "nz") field_ptrs.push_back({FieldPtr::Normal, 2, nullptr});
            else {
                field_ptrs.push_back({FieldPtr::Scalar, 0, cloud->getScalarField(f)});
            }
        }

        //循环写入
        char sep = params.separator;
        size_t num_points = cloud->size();
        Eigen::Vector3d shift = cloud->getGlobalShift();

        for (size_t i = 0; i < num_points; i++){
            const auto& p = cloud->m_xyz->points[i];

            for (size_t k = 0; k < field_ptrs.size(); k++){
                const auto& fp = field_ptrs[k];

                if (fp.type == FieldPtr::XYZ){
                    if (fp.sub_index == 0) file << (p.x + shift.x());
                    else if (fp.sub_index == 1) file << (p.y + shift.y());
                    else file << (p.z + shift.z());
                }
                else if (fp.type == FieldPtr::RGB){
                    if (cloud->m_colors && i < cloud->m_colors->size()) {
                        const auto& c = (*cloud->m_colors)[i];
                        int val = 0;
                        if (fp.sub_index == 0) val = c.r;
                        else if (fp.sub_index == 1) val = c.g;
                        else val = c.b;
                        file << val;
                    } else file << 0;
                }
                else if (fp.type == FieldPtr::Normal) {
                    if (cloud->m_normals && i < cloud->m_normals->size()) {
                        Eigen::Vector3f n = (*cloud->m_normals)[i].get();
                        if (fp.sub_index == 0) file << n.x();
                        else if (fp.sub_index == 1) file << n.y();
                        else file << n.z();
                    } else file << 0;
                }
                else if (fp.type == FieldPtr::Scalar){
                    file << (*fp.scalar_vec)[i];
                }

                if (k < field_ptrs.size() - 1) file << sep;
            }
            file << "\n";
        }
        file.close();
        return true;
    }

    bool FileIO::savePCL(const Cloud::Ptr &cloud, const QString &filename, bool isBinary) {
        pcl::PCLPointCloud2 msg;
        msg.height = 1;
        msg.width = cloud->size();
        msg.is_dense = cloud->m_xyz->is_dense;
        msg.is_bigendian = false;

        // 动态定义字段 (Fields)
        pcl::PCLPointField f;
        int current_offset = 0;

        // [XYZ]
        f.name = "x"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;

        f.name = "y"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;

        f.name = "z"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
        msg.fields.push_back(f); current_offset += 4;

        if (cloud->hasRGB()) {
            f.name = "rgb"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
            msg.fields.push_back(f); current_offset += 4;
        }

        // Normal
        if (cloud->hasNormals()) {
            f.name = "normal_x"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
            msg.fields.push_back(f); current_offset += 4;
            f.name = "normal_y"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
            msg.fields.push_back(f); current_offset += 4;
            f.name = "normal_z"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
            msg.fields.push_back(f); current_offset += 4;
        }

        // Scalar Fields (自定义字段)
        QStringList fieldNames = cloud->getScalarFieldNames();
        for (const QString& name : fieldNames) {
            f.name = name.toStdString();
            f.offset = current_offset;
            f.datatype = pcl::PCLPointField::FLOAT32;
            f.count = 1;
            msg.fields.push_back(f);
            current_offset += 4;
        }

        msg.point_step = current_offset;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);
        uint8_t* ptr = msg.data.data();

        // 预取标量场指针
        std::vector<const std::vector<float>*> scalar_ptrs;
        for (const QString& name : fieldNames) {
            scalar_ptrs.push_back(cloud->getScalarField(name));
        }

        Eigen::Vector3d shift = cloud->getGlobalShift();
        float sx = (float)shift.x();
        float sy = (float)shift.y();
        float sz = (float)shift.z();

        size_t num_points = cloud->size();
        // 开启 OpenMP 加速写入
#pragma omp parallel for
        for (int i = 0; i < (int)num_points; ++i)
        {
            uint8_t* pt_ptr = ptr + i * msg.point_step;
            int offset = 0;

            // XYZ (还原大坐标)
            const auto& p = cloud->m_xyz->points[i];
            float x = p.x + sx;
            float y = p.y + sy;
            float z = p.z + sz;

            memcpy(pt_ptr + offset, &x, 4); offset += 4;
            memcpy(pt_ptr + offset, &y, 4); offset += 4;
            memcpy(pt_ptr + offset, &z, 4); offset += 4;

            // RGB
            if (cloud->hasRGB()) {
                std::uint32_t rgb_val = cloud->getPointColorForSave(i);
                memcpy(pt_ptr + offset, &rgb_val, 4);
                offset += 4;
            }

            // Normal
            if (cloud->hasNormals() && cloud->m_normals) {
                Eigen::Vector3f n = (*cloud->m_normals)[i].get();
                float nx = n.x(); float ny = n.y(); float nz = n.z();
                memcpy(pt_ptr + offset, &nx, 4); offset += 4;
                memcpy(pt_ptr + offset, &ny, 4); offset += 4;
                memcpy(pt_ptr + offset, &nz, 4); offset += 4;
            }

            // Scalar Fields
            for (size_t k = 0; k < scalar_ptrs.size(); ++k) {
                float val = (*scalar_ptrs[k])[i];
                memcpy(pt_ptr + offset, &val, 4);
                offset += 4;
            }
        }

        // 写入文件
        if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
            pcl::io::savePCDFile(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary);
            return true;
        }
        else {
            // 默认为 PLY
            pcl::PLYWriter writer;
            int res = writer.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary, false);
            return (res == 0);
        }
    }
}
