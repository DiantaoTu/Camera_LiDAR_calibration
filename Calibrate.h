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
    const int window_size;          // 标定时使用的窗口大小
    const int max_iteration;        // 每个窗口最多迭代次数
    const float prob_threshold;     // 超过这个值就认为标定是准确的，停止当前窗口内的迭代
    const float rot_step;           // 扰动时旋转的角度，度为单位
    const float trans_step;         // 扰动时平移的距离，米为单位
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
    Calibrate(const std::vector<Frame>& _frames, const std::vector<Velodyne>& _lidars, 
            const int _window_size=9, const int _max_iter=30, const float _prob_threshold=0.9,
            const float _rot_step=0.1, const float _trans_step=0.005);
    
    bool ExtractLidarFeatures();
    bool ExtractImageFeatures();
    bool StartCalibration();

    void SetInitCalibration(const Eigen::Matrix4f T_cl);

    bool SaveEdgeImage(std::string path);
    bool LoadEdgeImage(std::string path);
    const Eigen::Matrix4f GetCalibration() const;
};

#endif