
#include "Velodyne.h"
#include "Virtualization.h"

using std::atan2;
using std::cos;
using std::sin;
using namespace std;


double d2r = 3.14159265358979323846 / 180.0;
double r2d = 180.0 / 3.14159265358979323846;

std::vector<double> vertical_degree =       // HW_Garage_Rosbag   Pandar40P
            {15.0 * d2r, 11.0 * d2r, 8.0 * d2r, 5.0 * d2r, 3.0 * d2r, 2.0 * d2r,
            1.6777 * d2r, 1.3333 * d2r, 1.0 * d2r, 0.6777 * d2r, 0.3333 * d2r,
            0 * d2r, -0.3333 * d2r, -0.6777 * d2r, -1.0 * d2r, -1.3333 * d2r, 
            -1.67777 * d2r, -2.0 * d2r, -2.3333 * d2r, -2.6777 * d2r, -3.0 * d2r, 
            -3.3333 * d2r, -3.67777 * d2r, -4.0 * d2r, -4.3333 * d2r, -4.6777 * d2r, 
            -5.0 * d2r, -5.3333 * d2r, -5.67777 * d2r, -6.0 * d2r, -7.0 * d2r, 
            -8.0 * d2r, -9.0 * d2r, -10.0 * d2r, -11.0 * d2r, -12.0 * d2r, 
            -13.0 * d2r,-14.0 * d2r, -19.0 * d2r, -25.0 * d2r};


Velodyne::Velodyne(int scan, int _id, int _horizon_scan): 
                N_SCANS(scan),scanPeriod(0.1),id(_id),valid(true),horizon_scans(_horizon_scan),
                cloud(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_scan(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_discontinuity(new pcl::PointCloud<pcl::PointXYZI>)
{
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
}

Velodyne::Velodyne(): scanPeriod(0.1),valid(true),
                cloud(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_scan(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_discontinuity(new pcl::PointCloud<pcl::PointXYZI>)
{
    N_SCANS = 0;
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
}

Velodyne::~Velodyne()
{
  
}


void Velodyne::LoadLidar(string filePath)
{
    string::size_type pos = filePath.rfind('.');
    string type = filePath.substr(pos);
    if(type == ".ply")
    {
        if(pcl::io::loadPLYFile(filePath, *cloud) == -1)
        {
            LOG(ERROR) << "Fail to load lidar data at " << filePath << endl;
            return;
        }
    }
    else if(type == ".pcd")
    {    
        if(pcl::io::loadPCDFile(filePath, *cloud) == -1)
        {
            LOG(ERROR) << "Fail to load lidar data at " << filePath << endl;
            return;
        }
    }
    else
    {
        LOG(ERROR) << "unknown point cloud format, only .ply or .pcd" << endl;
        return;
    }
    name = filePath;
    //从点云中移除NAN点也就是无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    *cloud = removeClosedPointCloud(0.5);

    // 改变LiDAR的坐标系，原本是X向右，Y向前，Z向上
    // 变成 X向右，Y向下，Z向前，这种顺序适应相机的坐标系
    // Eigen::Matrix4f T_cam_lidar;
    // T_cam_lidar << 1, 0, 0, 0 ,
    //                0, 0, -1, 0,
    //                0, 1, 0, 0,
    //                0, 0, 0, 1;
    // pcl::transformPointCloud(*cloud, *cloud, T_cam_lidar);

}

// 移除距离坐标系原点太近的点
pcl::PointCloud<pcl::PointXYZI> Velodyne::removeClosedPointCloud(float threshold)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    float sq_threshold = threshold * threshold;
    cloud_out.points.resize(cloud->points.size());
    size_t j = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        float dis = cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y + cloud->points[i].z * cloud->points[i].z;
        if (dis < sq_threshold)
            continue;
        cloud_out.points[j] = cloud->points[i];
        j++;
    }
    if (j != cloud->points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;

    return cloud_out;
}

// 对点云进行重新排序，从A-LOAM里抄的
void Velodyne::ReOrderVLP()
{
    scanStartInd.clear();
    scanEndInd.clear();
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    
    // cout << "scanRengistratione \n";
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        LOG(ERROR) << "only support velodyne with 16, 32 or 64 scan line!\n";
        return;
    }

    double horizon_resolution = 2.0 * M_PI / horizon_scans;
    
    
    int cloudSize = cloud->points.size();
    
    float startOri = -atan2(cloud->points[0].y, cloud->points[0].x);
    float endOri = -atan2(cloud->points[cloudSize - 1].y,
                          cloud->points[cloudSize - 1].x) +
                   2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.intensity = cloud->points[i].intensity;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            
        }
        //printf("angle %f scanID %d \n", angle, scanID);

    

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        double time = ori - startOri;

        int row_index = scanID;
        int col_index = round((ori - startOri) / horizon_resolution);
       

        point.intensity = scanID;
        
        laserCloudScans[scanID].push_back(point); 
        // cloud_scan_count++;
        // point_idx_to_image[cloud_scan_count] = pair<size_t, size_t>(row_index, col_index);
        // image_to_point_idx[pair<size_t, size_t>(row_index, col_index)] = cloud_scan_count;
        // float relTime = (ori - start_ori) / (end_ori - start_ori);

    }
    cloudSize = count;
    
    // 记录每个scan的起始点和终止点的索引
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = cloud_scan->size();
        *cloud_scan += laserCloudScans[i];
        scanEndInd[i] = cloud_scan->size() - 1;
    }
    // 清空cloud
    // std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>().swap((*cloud).points);
    // cloud->width = 0;
    // cloud->height = 0;
    // cloud->clear();
    // 可视化range image，只用于debug
    // {
    //     cv::Mat img_depth = cv::Mat::zeros(range_image.rows(), range_image.cols(), CV_8UC3);
    //     for(int i = 0; i < img_depth.rows; i++)
    //         for(int j = 0; j < img_depth.cols; j++)
    //         {
    //             float max_depth = 5;
    //             float real_depth = range_image(i,j);
    //             if(real_depth > max_depth)
    //                 real_depth = max_depth;
    //             uchar relative_depth = static_cast<uchar>(real_depth / max_depth * 255.0);
    //             if(real_depth == 0)
    //                 img_depth.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
    //             else 
    //                 img_depth.at<cv::Vec3b>(i,j) = Gray2Color(relative_depth);
    //         }
    //     cv::imwrite("range_image.jpg", img_depth);
    // }
}


// 提取特征点，只提取每条扫描线上的深度不连续的点
void Velodyne::ExtractFeatures()
{
    if(cloud_scan->empty())
    {
        LOG(ERROR) << "cloud_scan is empty, call Reorder first" << endl;
        return;
    }

    int cloudSize = cloud_scan->points.size();

    // 计算每个点到原点的距离, 保存成一个数组是为了后面方便
    float* cloudDistance;
    cloudDistance = (float*)malloc(cloudSize * sizeof(float));
    for(size_t i = 0; i < cloudSize; i++)
    {
        cloudDistance[i] = sqrt(cloud_scan->points[i].x * cloud_scan->points[i].x + 
                                cloud_scan->points[i].y * cloud_scan->points[i].y + 
                                cloud_scan->points[i].z * cloud_scan->points[i].z);
    }

    
    // 遍历所有的scan，找到深度不连续的点
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        for(int idx = scanStartInd[i] + 1; idx < scanEndInd[i] - 1; idx++)
        {
            PointType point = cloud_scan->points[idx];
            float discontinue = max(cloudDistance[idx - 1] - cloudDistance[idx], 
                            max(cloudDistance[idx + 1] - cloudDistance[idx], 0.f));
            if(discontinue < 1.0)
                continue;
            discontinue = sqrt(discontinue);
            point.intensity = discontinue;
            cloud_discontinuity->push_back(point);
        }
    }

}

void Velodyne::SetName(std::string _name)
{
    name = _name;
}

void Velodyne::SaveFeatures(std::string path)
{
    string base_name(name);
    base_name = base_name.substr(0, base_name.rfind('.'));  // aaa/bbb/ccc.pcd -> aaa/bbb/ccc
    base_name = base_name.substr(base_name.rfind('/') + 1); // aaa/bbb/ccc -> ccc

    if(!cloud_scan->points.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_cloud_scan.pcd", *cloud_scan);
    }
    if(!cloud_discontinuity->points.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_cloud_edge.pcd", *cloud_discontinuity);
    }
}


