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

    for(int i = 0; i < image_names.size(); i++)
    {
        Frame frame(image_names[i], i);
        frame.EdgeFilter();
        frame.InverseDistanceTransform();
    }

}