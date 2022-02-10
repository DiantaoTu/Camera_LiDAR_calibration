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

int main(int argc, char** argv)
{
    string lidar_data_path = "/home/tdt/Data_tdt/kitti_00/images/lidar/";
    string image_data_path = "/home/tdt/Data_tdt/kitti_00/images/image_2/";
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
    K << 718.856, 0, 607.1928,
        0, 718.856, 185.2157,
        0, 0, 1;
    Eigen::Matrix4f T_cl;
    // 从kitti配置文件里计算得到的T_cl的GT
    T_cl <<  4.27680239e-04, -9.99967248e-01, -8.08449168e-03, 4.79539786e-02,
            -7.21062651e-03,  8.08119847e-03, -9.99941316e-01, -5.51710332e-02,
            9.99973865e-01,  4.85948581e-04, -7.20693369e-03, -2.88417104e-01,
            0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00;

    Eigen::AngleAxisf angleAxis_cl(T_cl.block<3,3>(0,0));
    Eigen::Vector3f t_cl = T_cl.block<3,1>(0,3);
    cout << angleAxis_cl.axis().x() << " " << angleAxis_cl.axis().y() << " " << angleAxis_cl.axis().z() << " " 
        << angleAxis_cl.angle() << endl;
    cout << t_cl.transpose() << endl;

    vector<Frame> frames;
    for(int i = 0; i < image_names.size(); i++)
    {
        if(i >= 200)
            break;
        Frame f(image_names[i], i, K);
        frames.push_back(f);
        
    }
    vector<Velodyne> lidars;
    for(int i = 0; i < lidar_names.size(); i++)
    {
        if(i >= 200)
            break;
        Velodyne l(64, i);
        l.SetName(lidar_names[i]);
        lidars.push_back(l);
        
    }

    Calibrate calib(frames, lidars, 1);
    calib.SetInitCalibration(T_cl);

    calib.ExtractLidarFeatures();

    calib.ExtractImageFeatures();
    calib.SaveEdgeImage("./edge_images/");
    calib.LoadEdgeImage("./edge_images/");
    return 0;

    calib.StartCalibration();
    // t1 = chrono::high_resolution_clock::now();
    // for(int i = 0; i < image_names.size(); i++)
    // {
    //     Frame frame(image_names[i], i);
    //     frame.EdgeFilter();
    //     frame.InverseDistanceTransform(100, 3);
    //     break;
    // }
    // t2 = chrono::high_resolution_clock::now();
    // cout << "time spent: " << chrono::duration_cast<chrono::duration<double> >(t2 - t1).count() << " s" << endl;

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