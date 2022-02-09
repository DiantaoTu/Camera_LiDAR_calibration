/*
 * @author: TuDian tao
 * @Date: 2021-04-25 10:28:53
 * @LastEditTime: 2022-02-09 16:18:44
 */


#ifndef LIDAR_DATA_H
#define LIDAR_DATA_H
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <sstream> 
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> 
#include <pcl/common/transforms.h> 
#include <pcl/registration/icp.h>
#include <cmath>
#include <stdio.h> 
#include <stdlib.h>
#include <stack>

#include <ctime>
#include <cstdlib>
#include <chrono>
#include <glog/logging.h>
#include "common.h"

#pragma once

typedef pcl::PointXYZI PointType;

class Velodyne
{
public:
    std::string name;   // 文件名称
    int N_SCANS;        // 雷达线数
    int horizon_scans;  // 水平扫描的列数
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;  // 从文件读取的雷达数据
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_scan;   // 把雷达数据按scan排序
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_discontinuity;   // 深度不连续性

    int id;
    

    double scanPeriod;
    bool valid;

private:
	// 保存每个scan开始和结束的点在laserCloud中的index，而且每个scan的前5个点和后六个点都被剔除了
	std::vector<int> scanStartInd, scanEndInd;

    pcl::PointCloud<pcl::PointXYZI> removeClosedPointCloud(float threshold);

    /**
     * @description: 提取边缘特征，并把提取出的特征点的位置用一个图像来表示
     * @param picked_image {MatrixXf&} 用来表示提取的特征点的位置，每个像素对应一个特征点，像素值就是深度，和range image 一样
     * @param max_curvature {float} 最大的曲率，超过这个范围的曲率就视为外点
     * @param intersect_angle_threshold {float} 当前点与它所处面片的夹角，具体看livox loam里的公式4，夹角大于这个值就跳过
     * @return {*}
     */    
public:

    // scan>0 代表是VLP雷达
    Velodyne(int scan, int _id, int _horizon_scan = 1800);
    Velodyne();
    ~Velodyne();
    
    // 从本地读取雷达
    void LoadLidar(std::string filePath);
    // 对输入的雷达点重新按照scan进行排序,只用于VLP雷达
    void ReOrderVLP();		
    // 对输入的雷达点重新按照scan进行排序,目前用于华为的地下车库数据
    void ReOrder();         
    // 提取特征点  
    void ExtractFeatures();	
    // 设置雷达的名字，也就是对应的雷达点云保存的位置
    void SetName(std::string name);
    // 保存特征点到本地，用于debug
    void SaveFeatures(std::string path);

};



#endif