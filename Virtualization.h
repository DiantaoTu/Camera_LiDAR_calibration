/*
 * @Author: Diantao Tu
 * @Date: 2021-10-22 17:15:42
 */

#ifndef _VIRTUALIZATION_H_
#define _VIRTUALIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> 
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include "common.h"
#include "Geometry.hpp"


using namespace std;

//灰度图转为彩虹图:灰度值255~0分别对应：红、橙、黄、绿、青、蓝。
cv::Vec3b Gray2Color(uchar gray);


// 把深度图变为彩色图，最大深度和最小深度为-1时代表使用原图的最大深度以及0作为深度的上下限
cv::Mat DepthImageRGB(const cv::Mat& depth_map, const float max_depth = -1, const float min_depth = -1);

// 把深度图和彩色图结合到一起，有深度的时候显示深度，没深度的时候显示彩色图像
cv::Mat CombineDepthWithRGB(const cv::Mat& depth_image, const cv::Mat& rgb_image, 
                            const float max_depth = 10, const float min_depth = 0);

void SaveDepthImageRaw(const cv::Mat& depth_image, const std::string file_path);

/**
 * @description: 把雷达点投影到图像上
 * @param {cloud} 要投影的点云
 * @param {image} 点云要投影的图像，必须是单通道的灰度图
 * @param {K} 内参
 * @param {T_cl} 点云到相机的变换
 * @return {cv::Mat} 投影结果，三通道灰度图，点云的投影点用红色表示
 */
template<typename T>
cv::Mat ProjectLidar2ImageGray(const pcl::PointCloud<T> cloud, const cv::Mat image, 
                        const Eigen::Matrix3f K, const Eigen::Matrix4f T_cl)
{
    if(image.channels() != 1)
    {
        cout << "input image is not gray scale" << endl;
        return cv::Mat() ;
    }
    // 把单通道的灰度图变成三通道的灰度图，这是因为投影的点要用彩色表示，需要三通道
    cv::Mat img_out;
    vector<cv::Mat> images = {image, image, image};
    cv::merge(images, img_out);
    bool high_res = (img_out.rows * img_out.cols > 1280 * 720);
    pcl::PointCloud<T> cloud_trans;
    pcl::transformPointCloud(cloud, cloud_trans, T_cl);
    for(T p:cloud_trans.points)
    {
        Eigen::Vector3f point(p.x, p.y, p.z);
        point = K * point;
        if(point[2] <= 0)
            continue;
        float real_depth = point[2];
        int u = ceil(point[0] / real_depth);
        int v = ceil(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,255);
        if(!high_res)
            continue;
        u = floor(point[0] / real_depth);
        v = floor(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,255);   

        u = ceil(point[0] / real_depth);
        v = floor(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,255);   

        u = floor(point[0] / real_depth);
        v = ceil(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = cv::Vec3b(0,0,255); 
    }
    return img_out;
}

/**
 * @description: 把雷达点投影到图像上，投影的结果根据深度用彩虹图表示
 * @param {max_depth} 投影的最大深度，超过max_depth的点一律用红色表示
 * @param {min_depth} 投影的最小深度，小于min_depth的点一律用蓝色表示
 * @return {cv::Mat} 投影结果
 */
template<typename T>
cv::Mat ProjectLidar2ImageRGB(const pcl::PointCloud<T> cloud, const cv::Mat& image, 
                        const Eigen::Matrix3f K, const Eigen::Matrix4f T_cl, 
                        const float min_depth = 0,const float max_depth = 10)
{
    cv::Mat img_out;
    if(image.channels() == 3)
        img_out = image.clone();
    else if(image.channels() == 1)
    {
        vector<cv::Mat> images = {image, image, image};
        cv::merge(images, img_out);
    }
    else
    {
        cout << "error : image channel is neither 1 nor 3" << endl;
        return cv::Mat();
    }

    const float depth_diff = max_depth - min_depth;
    bool high_res = (img_out.rows * img_out.cols > 1280 * 720);
    pcl::PointCloud<T> cloud_trans;
    pcl::transformPointCloud(cloud, cloud_trans, T_cl);
    for(const T& p:cloud_trans.points)
    {
        Eigen::Vector3f point(p.x, p.y, p.z);
        point = K * point;
        if(point[2] <= 0)
            continue;
        float real_depth = point[2];
        int u = ceil(point[0] / real_depth);
        int v = ceil(point[1] / real_depth);
        if(point[2] > max_depth)
            point[2] = max_depth;
        else if(point[2] < min_depth)
            point[2] = min_depth;
        uchar relative_depth = static_cast<unsigned char>((point[2]-min_depth) / depth_diff * 255);
        cv::Vec3b color = Gray2Color(relative_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = color;   // u,v 是坐标，那么相应的行列就是第v行第u列
        // if the image is high resolution, set the project point to 4 pixel for better virtualization
        if(!high_res)
            continue;
        u = floor(point[0] / real_depth);
        v = floor(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = color;   

        u = ceil(point[0] / real_depth);
        v = floor(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = color;   

        u = floor(point[0] / real_depth);
        v = ceil(point[1] / real_depth);
        if(u < img_out.cols && v < img_out.rows && v > 0 && u > 0)       
            img_out.at<cv::Vec3b>(v,u) = color;
    }
    return img_out;
}


#endif

