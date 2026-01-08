#include "base/fileio.h"
#include "base/fieldmappingdialog.h"

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

    void FileIO::loadPointCloud(const QString &filename) {
        TicToc time;
        time.tic();
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
        if (!is_success){
            emit loadCloudResult(false, cloud, time.toc());
            return;
        }

        if (!cloud->empty() && cloud->getGlobalShift() == Eigen::Vector3d::Zero()){
            PointXYZRGBN min_pt, max_pt;
            pcl::getMinMax3D(*cloud, min_pt, max_pt);

            std::cout << std::fixed << std::setprecision(4) << min_pt.x << " " << min_pt.y << " " << min_pt.z << std::endl;
            std::cout << std::fixed << std::setprecision(4) << max_pt.x << " " << max_pt.y << " " << max_pt.z << std::endl;
            const double THRESHOLD_XY = 10000.0; // XY 超过 1万 才移
            const double THRESHOLD_Z = 100000.0; // Z 超过 10万 才移 (通常高程不动)

            bool need_shift =
                    std::abs(min_pt.x) > THRESHOLD_XY || std::abs(max_pt.x) > THRESHOLD_XY ||
                    std::abs(min_pt.y) > THRESHOLD_XY || std::abs(max_pt.y) > THRESHOLD_XY ||
                    std::abs(min_pt.z) > THRESHOLD_Z || std::abs(max_pt.z) > THRESHOLD_Z;

            if (need_shift){
                double shift_x = (std::abs(min_pt.x) > THRESHOLD_XY) ? -std::floor(min_pt.x / 1000.0) * 1000.0 : 0.0;
                double shift_y = (std::abs(min_pt.y) > THRESHOLD_XY) ? -std::floor(min_pt.y / 1000.0) * 1000.0 : 0.0;
                double shift_z = (std::abs(min_pt.z) > THRESHOLD_Z) ? -std::floor(min_pt.z / 1000.0) * 1000.0 : 0.0;

                Eigen::Vector3d suggested_shift(shift_x, shift_y, shift_z);
                Eigen::Vector3d min_vec(min_pt.x, min_pt.y, min_pt.z);
                bool is_skipped = false;

                emit requestGlobalShift(min_vec, suggested_shift, is_skipped);

                if (!is_skipped){
                    // Cloud 对象存储的是 Origin = -Shift
                    cloud->setGlobalShift(-suggested_shift);

                    size_t n = cloud->size();
                    #pragma omp parallel for
                    for (int i = 0; i < n; i++){
                        cloud->points[i].x += static_cast<float>(suggested_shift.x());
                        cloud->points[i].y += static_cast<float>(suggested_shift.y());
                        cloud->points[i].z += static_cast<float>(suggested_shift.z());
                    }
                }
            }
        }

        //后处理
        if (!cloud->is_dense){
            cloud->removeInvalidPoints(); // 移除无效点并同步标量场
        }

        cloud->setId(fileInfo.baseName());
        cloud->setInfo(fileInfo);
        cloud->update(); //更新包围盒，统计信息

        //备份颜色
        cloud->backupColors();

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

        Cloud::Ptr cloud_to_save = cloud;
        //检查是否存在偏移
        Eigen::Vector3d shift = cloud->getGlobalShift();
        bool has_shift = (shift != Eigen::Vector3d::Zero());
        if (has_shift){
            cloud_to_save.reset(new Cloud(*cloud));

            size_t n = cloud_to_save->size();

            //还原坐标
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(n); i++){
                cloud_to_save->points[i].x += static_cast<float>(shift.x());
                cloud_to_save->points[i].y += static_cast<float>(shift.y());
                cloud_to_save->points[i].z += static_cast<float>(shift.z());
            }

            cloud_to_save->update();
        }

        if (suffix == "las" || suffix == "laz"){
            is_success = saveLAS(cloud_to_save, filename);
        }
        else if (suffix == "txt" || suffix == "xyz" || suffix == "csv"){
            is_success = saveTXT(cloud_to_save, filename);
        }
        else {
            is_success = savePCL(cloud_to_save, filename, isBinary);
        }

        if (is_success) emit saveCloudResult(true, filename, time.toc());
        else emit saveCloudResult(false, filename, time.toc());
    }

    bool FileIO::loadLAS(const QString &filename, Cloud::Ptr &cloud) {
        m_is_canceled = false;
        emit progress(0);

        LASreadOpener lasreadopener;
        lasreadopener.set_file_name(filename.toLocal8Bit().constData());
        LASreader* lasreader = lasreadopener.open();

        if (!lasreader) return false;

        // 判断颜色
        int fmt = lasreader->header.point_data_format;
        bool has_color = (fmt == 2 || fmt == 3 || fmt == 5 || fmt == 7);
        cloud->setHasRGB(has_color);

        std::vector<float> intensities;
        // 预分配内存优化速度
        if (lasreader->npoints > 0) {
            cloud->reserve(lasreader->npoints);
            intensities.reserve(lasreader->npoints);
        }

        long long total_points = lasreader->npoints;
        long long current_point = 0;

        while (lasreader->read_point())
        {
            // 取消检查
            if (m_is_canceled){
                lasreader->close();
                delete lasreader;
                return false;
            }

            //进度更新,每读取2%更新一次
            current_point++;
            if (total_points > 0 && current_point % (total_points / 50 + 1) == 0){
                emit progress((int)(current_point * 100 / total_points));
            }

            PointXYZRGBN p;
            p.x = lasreader->point.get_x();
            p.y = lasreader->point.get_y();
            p.z = lasreader->point.get_z();

            uint8_t r = 0, g = 0, b = 0;
            if (has_color) {
                r = lasreader->point.rgb[0] >> 8;
                g = lasreader->point.rgb[1] >> 8;
                b = lasreader->point.rgb[2] >> 8;
            }

            std::uint32_t rgb_val = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            p.rgb = *reinterpret_cast<float*>(&rgb_val);

            intensities.push_back(static_cast<float>(lasreader->point.get_intensity()));
            cloud->push_back(p);
        }

        if (!intensities.empty()) {
            cloud->addScalarField("Intensity", intensities);
        }

        lasreader->close();
        delete lasreader;

        // LAS 通常是 dense 的，除非有点在无穷远，这里设为 true
        cloud->is_dense = true;

        emit progress(100);

        return true;
    }

    bool FileIO::loadPLY_PCD(const QString &filename, Cloud::Ptr &cloud) {
        m_is_canceled = false;
        emit progress(0);

        // 注意：pcl::io::loadPLYFile 这是一个阻塞函数，PCL内部不支持进度回调。
        // 所以加载 Blob 的阶段，进度条只能卡在 0 或者给一个假进度 (或者设为繁忙模式)。
        // 只有到了后面的“手动提取”阶段，我们才能控制进度。
        pcl::PCLPointCloud2 blob;
        int res = -1;
        // 先初步读取ply信息到blob中
        if (filename.endsWith(".ply", Qt::CaseInsensitive))
            res = pcl::io::loadPLYFile(filename.toLocal8Bit().toStdString(), blob);
        else
            res = pcl::io::loadPCDFile(filename.toLocal8Bit().toStdString(), blob);

        if (res == -1) return false;

        emit progress(20);

        // 准备字段信息
        QList<ct::FieldInfo> fields_info;
        for (const auto &f: blob.fields) {
            fields_info.append({QString::fromStdString(f.name), getPCLFieldType(f.datatype)});
        }

        // 请求 UI, mapping_result是用户选择的映射结果，Key是原字段名，Value是映射后的字段名
        QMap<QString, QString> mapping_result;
        emit requestFieldMapping(fields_info, mapping_result);

        if (mapping_result.isEmpty()) return false; // 用户取消

        // 基础转换，XYZRGBN信息会自动读取到cloud中
        pcl::fromPCLPointCloud2(blob, *cloud);

        emit progress(40);

        // 提取自定义字段
        size_t num_points = blob.width * blob.height;

        struct ExtractInfo {
            QString targetType; // "Scalar", "ColorPacked", "NormalX"...
            QString saveName;  // 存入Map的key(仅Scalar用)
            int offset;
            int datatype;
        };
        std::vector<ExtractInfo> tasks;

        // 遍历bolb中提取的字段名，若存在映射表中的字段名，则获取其映射字段名
        for (const auto& f : blob.fields){
            QString fname = QString::fromStdString(f.name);
            if (mapping_result.contains(fname)){
                QString action = mapping_result[fname];

                if (action == "Scalar Field" || action == "Intensity"){
                    QString saveName = (action == "Intensity") ? "Intensity" : fname;
                    tasks.push_back({"Scalar", saveName, (int)f.offset, f.datatype});
                }
                else if (action == "Color (Packed)"){
                    tasks.push_back({"ColorPacked", "", (int)f.offset, f.datatype});
                }
            }
        }

        // 准备容器
        QMap<QString, std::vector<float>> scalar_data;
        for(auto& t : tasks){
            if (t.targetType == "Scalar") scalar_data[t.saveName].resize(num_points);
        }

        //遍历提取
        for (size_t i = 0; i < num_points; i++){
            if (m_is_canceled) return false;

            if (num_points > 0 && i % (num_points / 50 + 1) == 0){
                int p = 40 + (int)(i * 60 / num_points);
                emit progress(p);
            }

            const uint8_t *point_ptr = &blob.data[i * blob.point_step];

            for (const auto& t : tasks){
                if (t.targetType == "Scalar"){
                    scalar_data[t.saveName][i] = getFieldValueAsFloat(point_ptr + t.offset, t.datatype);
                }
                else if (t.targetType == "ColorPacked"){
                    // 处理打包颜色
                    uint32_t rgb_packed = 0;
                    if (t.datatype == pcl::PCLPointField::FLOAT32){
                        rgb_packed = *reinterpret_cast<const uint32_t*>(point_ptr + t.offset);
                    }
                    else if (t.datatype == pcl::PCLPointField::UINT32){
                        rgb_packed = *reinterpret_cast<const uint32_t*>(point_ptr + t.offset);
                    }
                    cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb_packed);
                }
            }
        }

        //存入标量场
        for (auto &t : tasks){
            if (t.targetType == "Scalar"){
                if (!cloud->hasScalarField(t.saveName)){
                    cloud->addScalarField(t.saveName, scalar_data[t.saveName]);
                }
            }
        }

        // 设置 hasRGB
        bool has_color = false;
        for (const auto& val : mapping_result){
            if (val.contains("Color") || val.contains("Red")) has_color = true;
        }
        cloud->setHasRGB(has_color);

        // PCL 加载的可能有 NaN
        cloud->is_dense = blob.is_dense;
        return true;
    }

    bool FileIO::loadTXT(const QString &filename, Cloud::Ptr &cloud) {
        m_is_canceled = false;
        emit progress(0);

        std::ifstream file(filename.toLocal8Bit().constData());
        if (!file.is_open()) return false;

        //获取文件大小
        file.seekg(0, std::ios::end);
        long long file_size = file.tellg();
        file.seekg(0, std::ios::beg);
        long long read_bytes = 0;

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

        QMap<QString, std::vector<float>> scalars;
        bool has_color = false;

        while (std::getline(file, line))
        {
            // 取消检测
            if (m_is_canceled) return false;

            //进度更新
            read_bytes += line.size() + 1;
            // 每1MB更新一次
            if (read_bytes % 102400 == 0){
                emit progress((int)(read_bytes * 100 / file_size));
            }

            if (line.empty()) continue;

            QString qline = QString::fromStdString(line);
            QStringList parts;
            if (params.separator == ' ') parts = qline.simplified().split(' ', QString::SkipEmptyParts);
            else parts = qline.split(params.separator);

            PointXYZRGBN p;
            uint8_t r=255, g=255, b=255;

            for(auto it = params.col_map.begin(); it != params.col_map.end(); ++it) {
                int idx = it.key();
                if (idx >= parts.size()) continue;

                float val = parts[idx].toFloat();
                QString type = it.value();

                if (type == "x") p.x = val;
                else if (type == "y") p.y = val;
                else if (type == "z") p.z = val;
                else if (type == "r") { r = (uint8_t)val; has_color=true; }
                else if (type == "g") { g = (uint8_t)val; has_color=true; }
                else if (type == "b") { b = (uint8_t)val; has_color=true; }
                else if (type != "ignore") {
                    scalars[type].push_back(val);
                }
            }

            std::uint32_t rgb_val = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
            p.rgb = *reinterpret_cast<float*>(&rgb_val);

            cloud->push_back(p);
        }

        // 添加标量场
        for(auto it = scalars.begin(); it != scalars.end(); ++it) {
            if (it.value().size() == cloud->size()) {
                cloud->addScalarField(it.key(), it.value());
            }
        }

        cloud->setHasRGB(has_color);
        cloud->is_dense = true; // 假设解析后没有NaN
        return true;
    }

    bool FileIO::loadGeneralPCL(const QString &filename, Cloud::Ptr &cloud) {
        int res = -1;
        if (filename.endsWith(".obj", Qt::CaseInsensitive))
            res = pcl::io::loadOBJFile(filename.toLocal8Bit().toStdString(), *cloud);
        else if (filename.endsWith(".ifs", Qt::CaseInsensitive))
            res = pcl::io::loadIFSFile(filename.toLocal8Bit().toStdString(), *cloud);

        if (res == -1) return false;

        // 默认假设有颜色
        cloud->setHasRGB(true);
        cloud->is_dense = cloud->is_dense;
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
            const auto& p = cloud->points[i];
            laspoint.set_x(p.x);
            laspoint.set_y(p.y);
            laspoint.set_z(p.z);

            // [RGB] 仅当格式支持且需要保存时写入
            if (cloud->hasRGB()) {
                // 直接获取原始颜色 (uint32)，避免了 float 转换误差和伪彩色干扰
                std::uint32_t rgb_val = cloud->getPointColorForSave(i);
                // 解包并转为 LAS 的 16位颜色
                laspoint.rgb[0] = ((rgb_val >> 16) & 0xFF) * 256;
                laspoint.rgb[1] = ((rgb_val >> 8) & 0xFF) * 256;
                laspoint.rgb[2] = (rgb_val & 0xFF) * 256;
            }

            // [Intensity]
            if (intensity_ptr) {
                // 截断为 uint16
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
            enum Type { XYZ, RGB, Scalar};
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
            else {
                field_ptrs.push_back({FieldPtr::Scalar, 0, cloud->getScalarField(f)});
            }
        }

        //循环写入
        char sep = params.separator;
        size_t num_points = cloud->size();

        for (size_t i = 0; i < num_points; i++){
            const auto& p = cloud->points[i];

            //预取颜色
            uint32_t rgb_val = 0;
            if (cloud->hasRGB()) rgb_val = cloud->getPointColorForSave(i);

            // 填充字段
            for (size_t k = 0; k < field_ptrs.size(); k++){
                const auto& fp = field_ptrs[k];

                if (fp.type == FieldPtr::XYZ){
                    if (fp.sub_index == 0) file << p.x;
                    else if (fp.sub_index == 1) file << p.y;
                    else file << p.z;
                }
                else if (fp.type == FieldPtr::RGB){
                    //解包颜色
                    int val = 0;
                    if (fp.sub_index == 0) val = (rgb_val >>16) & 0xFF; //R
                    else if (fp.sub_index == 1) val = (rgb_val >>8) & 0xFF; //G
                    else val = rgb_val & 0xFF; //  B
                    file << val;
                }
                else if (fp.type == FieldPtr::Scalar){
                    file << (*fp.scalar_vec)[i];
                }
                //分隔符
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
        msg.is_dense = cloud->is_dense;
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

        // [RGB] 按需添加
        if (cloud->hasRGB()) {
            f.name = "rgb"; f.offset = current_offset; f.datatype = pcl::PCLPointField::FLOAT32; f.count = 1;
            msg.fields.push_back(f); current_offset += 4;
        }

        // [Scalar Fields] 按需添加所有自定义字段
        QStringList fieldNames = cloud->getScalarFieldNames();
        for (const QString& name : fieldNames) {
            f.name = name.toStdString();
            f.offset = current_offset;
            f.datatype = pcl::PCLPointField::FLOAT32;
            f.count = 1;
            msg.fields.push_back(f);
            current_offset += 4;
        }

        // 填充数据 Blob
        msg.point_step = current_offset;
        msg.row_step = msg.point_step * msg.width;
        msg.data.resize(msg.row_step);
        uint8_t* ptr = msg.data.data();

        // 预取指针加速
        std::vector<const std::vector<float>*> scalar_ptrs;
        for (const QString& name : fieldNames) {
            // 注意：cloud->getScalarField 返回的是 vector 指针，这里直接存储
            scalar_ptrs.push_back(cloud->getScalarField(name));
        }

        size_t num_points = cloud->size();
        for (size_t i = 0; i < num_points; ++i)
        {
            const auto& p = cloud->points[i];

            // 写入 XYZ
            memcpy(ptr, &p.x, 12); // float*3 = 12 bytes
            int internal_offset = 12;

            // 写入 RGB
            if (cloud->hasRGB()) {
                // 直接取原始颜色 (uint32)
                std::uint32_t rgb_val = cloud->getPointColorForSave(i);
                memcpy(ptr + internal_offset, &rgb_val, 4);
                internal_offset += 4;
            }

            // 写入 自定义字段
            for (size_t k = 0; k < scalar_ptrs.size(); ++k) {
                float val = (*scalar_ptrs[k])[i];
                memcpy(ptr + internal_offset, &val, 4);
                internal_offset += 4;
            }

            ptr += msg.point_step;
        }

        // 写入文件
        if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
            pcl::io::savePCDFile(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary);
            return true;
        }
        else {
            // 默认为 PLY
            pcl::PLYWriter writer;
            // 最后一个参数 false 表示不写入 camera 元素，避免那个 warning
            int res = writer.write(filename.toLocal8Bit().toStdString(), msg, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), isBinary, false);
            return (res == 0);
        }
    }
}
