/*
 * @Author: Diantao Tu
 * @Date: 2022-02-10 12:53:29
 */
#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <glog/logging.h>
#include <vector>
#include <omp.h>

#include "Frame.h"
#include "Velodyne.h"

class Calibrate
{
private:
    const int window_size;
    std::vector<Frame> frames;
    std::vector<Velodyne> lidars;
    Eigen::Matrix4f init_T_cl;
    Eigen::Matrix4f final_T_cl;
    /**
     * @description: 对外参进行扰动，分别在xyz三个轴上进行旋转和平移，共有3^6=729种扰动结果
     * @param T_cl 要扰动的外参
     * @param rotation_step {float} 旋转的角度，以度为单位
     * @param translation_step {float} 平移的距离，以米为单位
     * @return {*} 所有扰动之后的结果
     */    
    eigen_vector<Eigen::Matrix4f> PerturbCalibration(const Eigen::Matrix4f& T_cl, 
                const float rotation_step, const float translation_step);
    // 计算Jc，论文中的公式3
    double ComputeJc(const pcl::PointCloud<pcl::PointXYZI>& cloud, Frame& frame, const Eigen::Matrix4f& T_cl);
    // 计算当前的外参是一个“正确”的外参的概率
    double CorrectProbability(double Fc);

public:
    Calibrate(const std::vector<Frame>& _frames, const std::vector<Velodyne>& _lidars, const int _window_size=9);
    
    bool ExtractLidarFeatures();
    bool ExtractImageFeatures();
    bool StartCalibration();

    void SetInitCalibration(const Eigen::Matrix4f T_cl);

    bool SaveEdgeImage(std::string path);
    bool LoadEdgeImage(std::string path);
    const Eigen::Matrix4f GetCalibration() const;
};

#endif