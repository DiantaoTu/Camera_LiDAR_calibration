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

using namespace std;

int main(int argc, char** argv)
{
    string lidar_data_path = "/home/tdt/Data_tdt/kitti_00/images/lidar/";
    string image_data_path = "/home/tdt/Data_tdt/kitti_00/images/image_2/";

    vector<string> lidar_names, image_names;
    IterateFiles(lidar_data_path, lidar_names, ".pcd");
    sort(lidar_names.begin(), lidar_names.end(), FileNameComp);
    IterateFiles(image_data_path, image_names, ".png");
    sort(image_names.begin(), image_names.end(), FileNameComp);

    // for(int i = 0; i < lidar_names.size(); i++)
    // {
    //     Velodyne lidar(64, i);
    //     lidar.LoadLidar(lidar_names[i]);
    //     lidar.ReOrderVLP();
    //     lidar.ExtractFeatures();
    //     lidar.SaveFeatures("./");
    // }

    // 两个计时器
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = chrono::high_resolution_clock::now();
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

    

}