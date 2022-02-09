/*
 * @Author: Diantao Tu
 * @Date: 2021-11-19 13:50:16
 */

#ifndef _FILE_IO_H_
#define _FILE_IO_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <glog/logging.h>
#include <omp.h>
#include "common.h"
#include "Frame.h"

// 字符串分割
std::vector<std::string> SplitString(const std::string& str, const char& delimma);

/**
 * @description: 从文件中读取位姿并保存,位姿是以变换矩阵的顺序存储的,也就是12个数,R(0,0) R(0,1) R(0,2) t(0) R(1,0) ... t(1) R(2,0) ... t(2)
 * @param file_path 位姿文件保存的路径
 * @param rotation_list 输出的旋转矩阵
 * @param trans_list 输出的平移向量
 * @param name_list 输出的文件名(如果有的话)
 */
void ReadPoseT(std::string file_path, eigen_vector<Eigen::Matrix3d>& rotation_list, 
            eigen_vector<Eigen::Vector3d>& trans_list, std::vector<std::string>& name_list );

// 和上面的一样，只不过是以四元数和平移向量形式存储的
// qx qy qz qw tx ty tz
void ReadPoseQt(std::string file_path, eigen_vector<Eigen::Matrix3d>& rotation_list, 
            eigen_vector<Eigen::Vector3d>& trans_list, std::vector<std::string>& name_list );

void ExportPoseT(const std::string file_path, const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& rotation_list,
                const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& trans_list,
                const std::vector<std::string>& name_list);

#endif