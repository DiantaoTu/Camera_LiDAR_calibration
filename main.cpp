/*
 * @Author: Diantao Tu
 * @Date: 2022-02-09 16:10:41
 */
#include <vector>
#include <string>
#include <iostream>

#include "common.h"
#include "Velodyne.h"
#include "Frame.h"
#include "Calibrate.h"

using namespace std;

// 设置log所在的路径
void SetLog(string log_path);
// 计算两个标定结果之间的差异
void CalibrationError(const Eigen::Matrix4f& T_cl1, const Eigen::Matrix4f& T_cl2);

int main(int argc, char** argv)
{
    string lidar_data_path = "/home/tdt/Data_tdt/kitti_03/images/lidar/";
    string image_data_path = "/home/tdt/Data_tdt/kitti_03/images/image_2/";
    string edge_image_path = "/home/tdt/Data_tdt/kitti_03/other_images/edge_images/";
    string log_path = "./log/";
    SetLog(log_path);

    vector<string> lidar_names, image_names;
    IterateFiles(lidar_data_path, lidar_names, ".pcd");
    sort(lidar_names.begin(), lidar_names.end(), FileNameComp);
    IterateFiles(image_data_path, image_names, ".png");
    sort(image_names.begin(), image_names.end(), FileNameComp);

    // 两个计时器
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = chrono::high_resolution_clock::now();
    
    // 相机内参
    Eigen::Matrix3f K;
    // 从kitti配置文件里计算得到的T_rl的GT, 也就是从雷达到参考坐标系
    Eigen::Matrix4f T_rl;
    // image_2对应的投影矩阵 P = K[R|t]
    Eigen::Matrix<float, 3, 4> P2;
    // kitti 00-02
    K << 718.856, 0, 607.1928,
        0, 718.856, 185.2157,
        0, 0, 1;
    T_rl << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02, 
            -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02, 
            9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
            0, 0, 0, 1;
    P2 << 7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02, 4.538225000000e+01, 
          0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02, -1.130887000000e-01, 
          0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 3.779761000000e-03;
    // kitti 03
    // K << 721.5377, 0, 609.559,
    //     0, 721.5377, 172.854,
    //     0, 0, 1;
    // T_rl << 2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02, -2.796816941295e-03, 
    //         1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01, -7.510879138296e-02, 
    //         9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02, -2.721327964059e-01,
    //         0, 0, 0, 1;
    // P2 << 7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 4.485728000000e+01, 
    //       0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 2.163791000000e-01, 
    //       0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 2.745884000000e-03;
    // kitti 04-12
    // K << 707.0912, 0, 601.887,
    //     0, 707.0912, 183.11,
    //     0, 0, 1;
    // T_rl << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03, 
    //         -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02, 
    //         9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
    //         0, 0, 0, 1;
    // P2 << 7.070912000000e+02, 0.000000000000e+00, 6.018873000000e+02, 4.688783000000e+01, 
    //       0.000000000000e+00, 7.070912000000e+02, 1.831104000000e+02, 1.178601000000e-01, 
    //       0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00, 6.203223000000e-03;    

    // [R|t]
    Eigen::Matrix<float, 3, 4> Rt = K.inverse() * P2;
    // 从参考坐标系到当前相机坐标系的变换
    Eigen::Matrix4f T_cr = Eigen::Matrix4f::Identity();
    T_cr.block<3,4>(0,0) = Rt;
    // 从雷达坐标系到当前相机坐标系的变换，也就是雷达的外参
    Eigen::Matrix4f T_cl = T_cr * T_rl;
    // 输出一下GT对应的轴角和平移，方便以后使用
    Eigen::AngleAxisf angleAxis_cl(T_cl.block<3,3>(0,0));
    Eigen::Vector3f t_cl = T_cl.block<3,1>(0,3);
    LOG(INFO) << "ground truth calibration: ";
    LOG(INFO) << "angle axis: " << angleAxis_cl.axis().x() << " " << angleAxis_cl.axis().y() << " " 
            << angleAxis_cl.axis().z() << " " << angleAxis_cl.angle() << endl;
            
    LOG(INFO) << "translation: " << t_cl.x() << " " << t_cl.y() << " " << t_cl.z() << endl;

    // 对外参进行一定程度的扰动
    Eigen::Matrix4f delta_T = Eigen::Matrix4f::Identity();
    delta_T.block<3,3>(0,0) = Eigen::Matrix3f(Eigen::AngleAxisf(2.f * M_PI / 180.f, Eigen::Vector3f(0,0,1)));
    delta_T.block<3,1>(0,3) = Eigen::Vector3f(0.05, 0.05, -0.05);
    Eigen::Matrix4f T_cl_init = delta_T * T_cl;

    // 输出一下扰动后的外参
    angleAxis_cl = Eigen::AngleAxisf(T_cl_init.block<3,3>(0,0));
    t_cl = T_cl_init.block<3,1>(0,3);
    LOG(INFO) << "current calibration: ";
    LOG(INFO) << "angle axis: " << angleAxis_cl.axis().x() << " " << angleAxis_cl.axis().y() << " " 
            << angleAxis_cl.axis().z() << " " << angleAxis_cl.angle() << endl;
    LOG(INFO) << "translation: " << t_cl.x() << " " << t_cl.y() << " " << t_cl.z() << endl;

    // 读取图像
    vector<Frame> frames;
    for(int i = 0; i < image_names.size(); i++)
    {
        if(i >= 350)
            break;
        Frame f(image_names[i], i, K);
        frames.push_back(f);
    }
    // 读取雷达
    vector<Velodyne> lidars;
    for(int i = 0; i < lidar_names.size(); i++)
    {
        if(i >= 350)
            break;
        Velodyne l(64, i);
        l.SetName(lidar_names[i]);
        lidars.push_back(l);  
    }

    // 标定
    Calibrate calib(frames, lidars, 350);
    calib.SetInitCalibration(T_cl_init);

    calib.ExtractLidarFeatures();

    // calib.ExtractImageFeatures();
    // calib.SaveEdgeImage("./edge_images/");
    calib.LoadEdgeImage(edge_image_path);
  
    calib.StartCalibration();
    
    Eigen::Matrix4f T_cl_final = calib.GetCalibration();
    angleAxis_cl = Eigen::AngleAxisf(T_cl_final.block<3,3>(0,0));
    t_cl = T_cl_final.block<3,1>(0,3);
    LOG(INFO) << "final calibration: ";
    LOG(INFO) << "angle axis: " << angleAxis_cl.axis().x() << " " << angleAxis_cl.axis().y() << " " 
            << angleAxis_cl.axis().z() << " " << angleAxis_cl.angle() << endl;
    LOG(INFO) << "translation: " << t_cl.x() << " " << t_cl.y() << " " << t_cl.z() << endl;

    CalibrationError(T_cl, T_cl_final);

    return 0;
}

void SetLog(string log_path)
{
    if(!boost::filesystem::exists(log_path))
        boost::filesystem::create_directories(log_path);
    google::InitGoogleLogging("Mapping");
    google::SetLogDestination(google::GLOG_INFO, log_path.c_str());
    google::SetStderrLogging(google::GLOG_INFO);
    FLAGS_logbufsecs = 0;
    LOG(INFO) << "Save log file at " << log_path;
}

void CalibrationError(const Eigen::Matrix4f& T_cl1, const Eigen::Matrix4f& T_cl2)
{
    Eigen::Matrix3f diff_R = T_cl1.block<3,3>(0,0).transpose() * T_cl2.block<3,3>(0,0);
    float diff_angle = acos((diff_R.trace() - 1) / 2.f) * 180.f / M_PI;
    float diff_trans = (T_cl1.block<3,1>(0,3) - T_cl2.block<3,1>(0,3)).norm();
    LOG(INFO) << "rotation difference: " << diff_angle << " degree, translation difference " << diff_trans << " m";
}