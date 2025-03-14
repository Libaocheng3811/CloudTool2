#ifndef CLOUDTOOL2_COMMON_H
#define CLOUDTOOL2_COMMON_H

#include "base/exports.h"

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <QString>

namespace ct
{
    /**
     * @brief 将颜色从HSV格式转换为RGB格式
     * 在HSV模型中，颜色是通过三个分量表示的：色调（Hue），饱和度（Saturation）和明度（Value）
     * hue 表示色调，通常在0到360度之间，以表示不同的颜色（例如，0°是红色，120°是绿色，240°是蓝色）。
     */
    void CT_EXPORT HSVtoRGB(float h, float s, float v, float& r, float& g, float& b);

    /**
     * @brief 从给定的变换中提取欧拉角（内在旋转， ZYX约定）
     * @param roll 滚转角
     * @param pitch 俯仰角
     * @param yaw 偏航角
     */
    void CT_EXPORT getEulerAngles(const Eigen::Affine3f& t, float& roll, float& pitch, float& yaw);

    /**
     * @brief 从给定的变换中提取轴角（内在旋转，ZYX约定）
     */
    void CT_EXPORT getAngleAxis(const Eigen::Affine3f& t, float& angle, float& axisX, float& axisY, float& axisZ);

    /**
     * @brief 从给定的变换中提取x, y, z和欧拉角（内在旋转，ZYX约定）
     */
    void CT_EXPORT getTranslationAndEulerAngles(const Eigen::Affine3f& t, float& x, float& y, float& z, float& roll, float& pitch, float& yaw);

    /**
     * @brief 从给定的平移和欧拉角（内在旋转，ZYX约定）创建变换
     */
    void CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Affine3f& t);

    /**
     * @brief 将给定的变换转换为字符串
     * @param decimals 有效位数
     */
    QString CT_EXPORT getTransformationQString(const Eigen::MatrixXf& mat, int decimals);


    /**
     * @brief 从给定的平移和欧拉角（内在旋转，ZYX约定）创建变换
     */
    Eigen::Affine3f CT_EXPORT getTransformation(float x, float y, float z, float roll, float pitch, float yaw);
}


#endif //CLOUDTOOL2_COMMON_H
