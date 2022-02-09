/*
 * @Author: Diantao Tu
 * @Date: 2021-11-10 20:33:55
 */

#ifndef _GEOMETRY_H
#define _GEOMETRY_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "common.h"

/**
 * @description: 投影三维点到直线上
 * @param T 可以是cv::point3f  cv::point3d
 * @param point {Point3f} 三维点的坐标
 * @param line {Vec6f} 直线的表示形式 a,b,c,d,e,f   (x-a)/d=(y-b)/e=(z-c)/f 
 * @return {Point3f} 投影点的坐标
 */  
template<typename T>
inline T ProjectPoint2Line(const T point, const pcl::ModelCoefficients line)
{
    float x0 = line.values[0];
    float y0 = line.values[1];
    float z0 = line.values[2];
    float nx = line.values[3];
    float ny = line.values[4];
    float nz = line.values[5];
    float k = (nx * (point.x - x0) + ny * (point.y - y0) + nz * (point.z - z0)) / (nx * nx + ny * ny + nz * nz);
    return T(k * nx +x0, k * ny + y0, k * nz + z0);
}

/**
 * @description: 计算三维点到空间直线的距离，就是先把点投影到直线上，然后计算点和投影点的距离
 * @param point 三维点
 * @param line 空间直线的表达，前三个数是经过的点，后三个数是方向向量（不需要是单位向量）
 * @param T double float
 * @return 点到直线距离
 */
template<typename T>
inline T PointToLineDistance(const T* point, const T* line)
{
    T x0 = line[0];
    T y0 = line[1];
    T z0 = line[2];
    T nx = line[3];
    T ny = line[4];
    T nz = line[5];
    T k = (nx * (point[0] - x0) + ny * (point[1] - y0) + nz * (point[2] - z0)) / (nx * nx + ny * ny + nz * nz);
    T project_point[3] = {k * nx +x0, k * ny + y0, k * nz + z0};
    T distance = sqrt((project_point[0] - point[0]) * (project_point[0] - point[0]) + 
                       (project_point[1] - point[1]) * (project_point[1] - point[1]) +
                       (project_point[2] - point[2]) * (project_point[2] - point[2]) );
    return distance;
}

/**
 * @description: 判断一系列三维点是否能形成一条空间直线，根据的是这些点的PCA方向，详情见 公式推导.md
 * @param points 三维点
 * @param tolerance 这个值越大，对直线的要求就越严苛，也就是这些点要更“直”一些
 * @return 形成的直线的参数 a b c d e f , 其中a bc是直线的法向量，d e f是直线经过的点
 */
template<typename T>
Eigen::Matrix<T,6,1> FormLine(const eigen_vector<Eigen::Matrix<T, 3, 1>>& points, T tolerance)
{
    Eigen::Matrix<T,3,1> center(0, 0, 0);
    for (int i = 0; i < points.size(); i++)
    {
        center = center + points[i]; 
    }
    center = center / T(points.size());

    Eigen::Matrix<T,3,3> covMat = Eigen::Matrix<T,3,3>::Zero();
    for (int i = 0; i < points.size(); i++)
    {
        Eigen::Matrix<T, 3, 1> tmpZeroMean = points[i] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<T,3,3>> saes(covMat);

    // if is indeed line feature
    // note Eigen library sort eigenvalues in increasing order
    if(saes.eigenvalues()[2] > tolerance * saes.eigenvalues()[1])
    {
        Eigen::Matrix<T,3,1> unit_direction = saes.eigenvectors().col(2);
        Eigen::Matrix<T, 6, 1> line_coeff;
        line_coeff << unit_direction[0], unit_direction[1], unit_direction[2], center[0], center[1], center[2] ;
        return line_coeff;
    }
    else 
        return Eigen::Matrix<T,6,1>::Zero();
}

// 二维两点距离的平方
template<typename T>
inline T PointDistance(const T* p1, const T* p2)
{
    return (p1[0] - p2[0])*(p1[0]-p2[0]) + (p1[1] - p2[1])*(p1[1]-p2[1]) ;
}

template<typename T>
inline T PointDistance(const cv::Point_<T>& p1, const cv::Point_<T>& p2){
    return (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y);
}

template<typename T>
inline T PointDistance(const cv::Vec<T,4>& line)
{
    return (line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]) ;
}

inline float PointDistance(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2){
    return (p1.x - p2.x)*(p1.x - p2.x)+(p1.y - p2.y)*(p1.y - p2.y)+(p1.z - p2.z)*(p1.z - p2.z);
}



// 点到平面距离  plane -- 平面系数a b c d
inline float PointToPlaneDistance(const cv::Vec4f& plane, const cv::Point3f& point)
{
    return abs(plane[0]*point.x + plane[1]*point.y + plane[2]*point.z + plane[3]) / 
            sqrt(plane[0] * plane[0] +plane[1] *plane[1] + plane[2]*plane[2]);
}

// T = float  double
template<typename T>
inline T PointToPlaneDistance(const T* plane, const T* point)
{
    return abs(plane[0] * point[0] + plane[1]* point[1] + plane[2] * point[2] + plane[3]) /
            sqrt(plane[0] * plane[0] +plane[1] *plane[1] + plane[2]*plane[2]);
}

// 点投影到平面
inline cv::Point3f ProjectPointToPlane(const cv::Point3f& point, const cv::Vec4f& plane)
{
    float dis = PointToPlaneDistance(plane, point);
    cv::Point3f p;
    p.x = point.x - dis * plane[0];
    p.y = point.y - dis * plane[1];
    p.z = point.z - dis * plane[2];
    return p;
}

inline float VectorAngle(const cv::Point2f& v1_start, const cv::Point2f& v1_end, 
                        const cv::Point2f& v2_start, const cv::Point2f& v2_end){
    cv::Vec2f v1(v1_start.x - v1_end.x, v1_start.y - v1_end.y);
    cv::Vec2f v2(v2_start.x - v2_end.x, v2_start.y - v2_end.y);
    return acos( v1.dot(v2) / cv::norm(v1) / cv::norm(v2) );
}

template<typename T>
inline T VectorAngle(const cv::Point3_<T>& v1, const cv::Point3_<T>& v2)
{
    T norm1 = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    T norm2 = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    return acos( (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z) / norm1 / norm2 );
}

// 两个平面之间的夹角，由两个平面的法向量计算
// 两个平面之间的夹角永远是锐角
template<typename T>
inline float PlaneAngle(const T plane1, const T plane2){
    float norm1 = sqrt(plane1[0] * plane1[0] + plane1[1] * plane1[1] + plane1[2] * plane1[2] );
    float norm2 = sqrt(plane2[0] * plane2[0] + plane2[1] * plane2[1] + plane2[2] * plane2[2] );
    return acos( abs(plane1[0] * plane2[0] + plane1[1] * plane2[1] + plane1[2] * plane2[2]) / norm1 / norm2 );
}

template<typename T>
inline T PlaneAngle(const T* plane1, const T* plane2){
    T norm1 = sqrt(plane1[0] * plane1[0] + plane1[1] * plane1[1] + plane1[2] * plane1[2] );
    T norm2 = sqrt(plane2[0] * plane2[0] + plane2[1] * plane2[1] + plane2[2] * plane2[2] );
    return acos( abs(plane1[0] * plane2[0] + plane1[1] * plane2[1] + plane1[2] * plane2[2]) / norm1 / norm2 );
}

/**
 * @description: 对三维点进行旋转平移
 * @param T 可以是 cv::Point3f cv::Point3d
 * @param point 三维点
 * @param transformation 变换矩阵
 * @return 变换后的三维点
 */
template<typename T>
inline T TranslatePoint(const T point, Eigen::Matrix4f transformation)
{
    return T(point.x * transformation(0,0) + point.y * transformation(0,1) + point.z * transformation(0,2) + transformation(0,3),
            point.x * transformation(1,0) + point.y * transformation(1,1) + point.z * transformation(1,2) + transformation(1,3),
            point.x * transformation(2,0) + point.y * transformation(2,1) + point.z * transformation(2,2) + transformation(2,3));
}

/**
 * @description: 用点斜式表示一条直线
 * @param T cv::poin2f  cv::point2d
 * @param sp 直线的起点
 * @param ep 直线的终点
 * @return 直线的表达式 k,b, 如果返回的b=infinite，那么就说明这条直线垂直于x轴
 */
template<typename T>
inline T FormLine(const T& sp, const T& ep )
{
    if(abs(sp.x - ep.x) < 1e-6)
        return T(sp.x, std::numeric_limits<float>::infinity());
    float k = (sp.y - ep.y)/(sp.x - ep.x);
    float b = sp.y - k * sp.x;
    return T(k, b);
}

// 和上面的一样，只是用一个cv::vec 来存储起点和终点，其中前两维是起点，后两维是终点
template<typename T>
inline cv::Point_<T> FormLine(const cv::Vec<T,4>& line)
{
    float k = (line[1] - line[3])/(line[0] - line[2]);
    float b = line[1] - k * line[0];
    return T(k, b);
}


/**
 * @description: 投影二维点到直线上
 * @param T 可以是 cv::point2f  cv::point2d
 * @param sp 直线的起始点
 * @param ep 直线的终止点
 * @param point 要投影的二维点
 * @return 投影点
 */
template<typename T>
inline T ProjectPoint2Line2D(const T& sp, const T& ep, const T& point)
{
    if(abs(sp.x - ep.x) < 1e-6)
        return T(sp.x, point.y);
    float k = (sp.y - ep.y)/(sp.x - ep.x);
    float b = sp.y - k * sp.x;
    float x_proj = (k * (point.y - b) + point.x) / (k * k + 1);
    return T(x_proj, k * x_proj + b);
}

// 和上面的函数一样，只是这里的直线用cv::vec来表示，其中前两维是起点，后两维是终点
template<typename T>
inline cv::Point_<T> ProjectPoint2Line2D(const cv::Vec<T,4>& line, const cv::Point_<T>& point)
{
    if(abs(line[0] - line[2]) < 1e-6)
        return cv::Point_<T>(line[0], point.y);
    float k = (line[1] - line[3])/(line[0] - line[2]);
    float b = line[1] - k * line[0];
    float x_proj = (k * (point.y - b) + point.x) / (k * k + 1);
    return cv::Point_<T>(x_proj, k * x_proj + b);
}

template<typename T>
inline T PointToLineDistance2D(const cv::Vec<T,4>& line, const cv::Point_<T>& point)
{
    cv::Point_<T> point_projected = ProjectPoint2Line2D(line, point);
    return PointDistance(point, point_projected);
}

#endif