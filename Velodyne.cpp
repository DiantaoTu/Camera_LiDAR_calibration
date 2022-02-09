
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
                N_SCANS(scan), world(false),scanPeriod(0.1),id(_id),valid(true),horizon_scans(_horizon_scan),
                cloud(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_scan(new pcl::PointCloud<pcl::PointXYZI>)
{
    cornerSharp.clear();
    cornerLessSharp.clear();
    surfFlat.clear();
    surfLessFlat.clear();
    edge_segmented.clear();
    
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);

    R_wl = Eigen::Matrix3d::Zero();
    t_wl = std::numeric_limits<double>::infinity() * Eigen::Vector3d::Ones();
    T_wc_wl = Eigen::Matrix4d::Identity();      // kitti

}

Velodyne::Velodyne(): world(false),scanPeriod(0.1),valid(true),
                cloud(new pcl::PointCloud<pcl::PointXYZI>),
                cloud_scan(new pcl::PointCloud<pcl::PointXYZI>)
{
    N_SCANS = 0;
    cornerSharp.clear();
    cornerLessSharp.clear();
    surfFlat.clear();
    surfLessFlat.clear();
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
    // Malloc(100000);
    R_wl = Eigen::Matrix3d::Zero();
    t_wl = std::numeric_limits<double>::infinity() * Eigen::Vector3d::Ones();
    T_wc_wl = Eigen::Matrix4d::Identity();  // kitti
    cloudCurvature = NULL;
    cloudSortInd = NULL;
    cloudState = NULL;
}

Velodyne::~Velodyne()
{
    cornerSharp.clear();
    cornerLessSharp.clear();
    surfFlat.clear();
    surfLessFlat.clear();
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
    Eigen::Matrix4f T_cam_lidar;
    T_cam_lidar << 1, 0, 0, 0 ,
                   0, 0, -1, 0,
                   0, 1, 0, 0,
                   0, 0, 0, 1;
    pcl::transformPointCloud(*cloud, *cloud, T_cam_lidar);

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

void Velodyne::ReOrderVLP()
{
    point_idx_to_image.clear();
    image_to_point_idx.clear();
    scanStartInd.clear();
    scanEndInd.clear();
    if(world)
        LOG(INFO) << "data now in world coordinate, reorder is not accurate" << endl;
    // cout << "scanRengistratione \n";
    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        LOG(ERROR) << "only support velodyne with 16, 32 or 64 scan line!\n";
        return;
    }

    double horizon_resolution = 2.0 * M_PI / horizon_scans;
    range_image.resize(N_SCANS, horizon_scans);
    
    int cloudSize = cloud->points.size();
    
    // atan2 范围是 (-pi, pi]
    double start_ori = atan2(cloud->points[0].x, cloud->points[0].z);
    if(start_ori < 0)
        start_ori += 2 * M_PI;
    double end_ori = atan2(cloud->points[cloudSize - 1].x, cloud->points[cloudSize - 1].z) + 2 * M_PI;
    if(end_ori - start_ori > 3 * M_PI)
        end_ori -= 2 * M_PI;
    else if(end_ori - start_ori < M_PI)
        end_ori += 2 * M_PI;
    
    // VLP雷达点云的存储顺序并不是第0线，第1线，第2线，第3线......这样的顺序，而是跳着存储的
    // 第0线，第8线，第1线，第9线，第2线，第10线.......
    // 这样就可以根据线数确定当前是这一列扫描的第几根, 例如当前根据角度计算得到为第0线，那么第0线对应着是
    // 第0次扫描，根据角度得到第13线，那么第13线对应着是第11次扫描
    map<int, int> scan_order;
    if(N_SCANS == 16)
    {
        for(int i = 0; i <= 7; i++)
            scan_order[i] = 2 * i;
        for(int i = 8; i <= 15; i++)
            scan_order[i] = 2 * i - 15;
    }

    bool half_passed = false;
    bool circle_passed = false;
    int count = cloudSize;
    PointType point;
    // 记录每一个scan的点云，后期要把它合起来成为一个完整的点云
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    // 记录每个scan中第i个点属于range image的哪一列，因为可能第一行只有1750个点，但是理论上应该有1800个点，
    // 因此第i个点不一定是在第i列，中间是有间隔的
    // {point idx in current scan, col index}
    vector<vector<pair<size_t, size_t>>> point_idx_to_col(N_SCANS);
    int col_offset = 0;

    int last_col = 0, last_scan = -1;
    size_t cloud_scan_count = 0;
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.intensity = cloud->points[i].intensity;

        float angle = atan(-point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
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

    

        // 把角度都变换到 0-2pi
        double ori = atan2(point.x, point.z);
        if(ori < 0)
            ori += 2 * M_PI;
        // 这里有一个基本的假设，雷达扫描的初始位置（也就是雷达数据的第一个点）是在Z轴的正方向附近的，也就是说
        // 第一个点的Z坐标一定是正数，否则下面的方法是不能用的
        // 分为两种情况，1.当初始点在第一象限(x>0, z>0) 2. 当初始点在第四象限(x<0, z>0)

        // 这是第一种情况, 初始点在第一象限, 如果当前点到初始点的角度大于180度，就说明已经超过一半了
        // 在已经超过一半的情况下，如果出现了 ori < start 就说明当前扫描又回到了第一象限，那么此时就要在
        // 角度上 +2pi。在已经超过一半的情况下，如果出现了 oir-start < pi 就说明扫描线已经超越起始点了，
        // 扫描了整整一圈，那么此时也要 +2pi。
        // 这两种 +2pi的情况合在一起就就是判断条件 oir-start < pi
        if(ori < M_PI)
        {    
            if(!half_passed)
            {
                if(ori - start_ori > M_PI)
                    half_passed = true;
            }
            else if(half_passed && (ori - start_ori) < M_PI)
                ori += 2 * M_PI;
        }
        else 
        {
            // 这是第二种情况，初始点在第四象限，因为所有点的角度都变换到了0-2pi，所以绝大部分的ori是小于start的，
            // 所以需要在ori < start_ori 时 +2pi。如果已经完整的扫描完了一整圈，回到了初始位置，那么之后的所有点
            // 都要在原有基础上再额外+2pi
            if(ori < start_ori)
            {
                ori += 2 * M_PI;
                if(ori - start_ori > M_PI)
                    half_passed = true;
            }
            else if(half_passed && ori > start_ori)
            {
                ori += 2 * M_PI;
                circle_passed = true;
            }
            if(circle_passed && ori < start_ori)
                ori += 2 * M_PI;
        }
        double time = ori - start_ori;

        int row_index = scanID;
        int col_index = round((ori - start_ori) / horizon_resolution);
       

        point.intensity = scanID;
        
        laserCloudScans[scanID].push_back(point); 
        // cloud_scan_count++;
        // point_idx_to_image[cloud_scan_count] = pair<size_t, size_t>(row_index, col_index);
        // image_to_point_idx[pair<size_t, size_t>(row_index, col_index)] = cloud_scan_count;
        // float relTime = (ori - start_ori) / (end_ori - start_ori);

    }
    cloudSize = count;
    
    // 每个scan的前5个点和后6个点都不算
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = cloud_scan->size() + 5;
        *cloud_scan += laserCloudScans[i];
        scanEndInd[i] = cloud_scan->size() - 6;
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

void Velodyne::ReOrder()
{
    if(world)
        cout << "data now in world coordinate, reorder is not accurate" << endl;
    
    // pcl::PointCloud<PointType> cloud_tmp;
    pcl::copyPointCloud(*cloud, *cloud_scan);
    for(int i = 0; i < cloud->points.size(); i++)
    {
        double sqrt_xy = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
        double vertical_angle = atan2(cloud->points[i].z, sqrt_xy);
        cloud_scan->points[i].intensity = vertical_angle;   // intensity记录扫描线与xy平面夹角
    }

    int same_ring_num = 0;
    int ring = 0;
    double last_angle = -1;     // 前一个点与水平方向的夹角
    // 经过下面这些操作后,cloud_scan中的点还是按列排序的,但每个点都有了ring
    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(cloud_scan->points[i].intensity > last_angle)
            ring = 0;

        for(; ring < vertical_degree.size(); ring++)
            if(cloud_scan->points[i].intensity >= vertical_degree[ring])   
                break;
        if(ring > N_SCANS-1)   ring = N_SCANS - 1;
        if(ring != 0)
        {
            double diff1 = abs(vertical_degree[ring-1] - cloud_scan->points[i].intensity);
            double diff2 = abs(vertical_degree[ring] - cloud_scan->points[i].intensity);
            // 如果diff1<diff2,说明扫描线更靠近第ring-1条,应该ring = ring - 1
            ring = diff1 > diff2 ? ring : ring - 1;
        }
        last_angle = cloud_scan->points[i].intensity;   // 记录一下夹角
        // ring++;     // 最终的ring是从1开始的,而且连续两条扫描线不能在同一个ring上,自增1恰好可以避免这种情况
        cloud_scan->points[i].intensity = ring;
        if(i == 0) continue;

        // 以下部分是对ring进行一定修正, 有些时候由于噪声影响,会出现同一列上好几个点都属于同一个ring
        // 这是明显错误的,要进行修正. 对于连续的n个点有相同的ring, 那么最后一个点ring不变, 之前的
        // 点ring-1, 再之前的ring-2, 一直到 ring-n+1
        if(cloud_scan->points[i].intensity == cloud_scan->points[i-1].intensity)
            same_ring_num ++;
        if((cloud_scan->points[i].intensity != cloud_scan->points[i-1].intensity) && (same_ring_num > 0))
        {
            for(int idx = 1; idx < same_ring_num; idx++)
                cloud_scan->points[i - 1 - idx].intensity -= idx;
            same_ring_num = 0;
        }
    }

    
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for(int i = 0; i < cloud_scan->points.size(); i++)
    {
        ring = cloud_scan->points[i].intensity;
        if(ring > N_SCANS - 10 || ring < 5)
            continue;
        laserCloudScans[ring].push_back(cloud_scan->points[i]);
    }
    cloud_scan->points.clear();
    // 每个scan的前5个点和后6个点都不算
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = cloud_scan->size() + 5;
        *cloud_scan += laserCloudScans[i];
        scanEndInd[i] = cloud_scan->size() - 6;
    }
    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>>().swap((*cloud).points);
    cloud->width = 0;
    cloud->height = 0;
    cloud->clear();
}

// 提取特征点，完成后cornerSharp  cornerLessSharp  surfFlat  surfLessFlat 就都有值了
// 这段是从 A-LOAM 里抄来的
void Velodyne::ExtractFeatures(float max_curvature, float intersect_angle_threshold, bool double_extract)
{
    if(cloud_scan->empty())
    {
        LOG(ERROR) << "cloud_scan is empty, call Reorder first" << endl;
        return;
    }

    int cloudSize = cloud_scan->points.size();
    Segmentation();
    if(cloud_scan->points.size() < cloudSize * 0.1)
    {
        LOG(WARNING) << "LiDAR data " << id << " has something wrong";
        valid = false;
        return;
    }
    cloudSize = cloud_scan->points.size();
    size_t neighbor_size = 5;

    cloudCurvature = (float*)malloc(cloudSize * sizeof(float));
    // 把曲率初始化成一个较大的数字，后面会有略过曲率太大的点，所以如果某些点没有被计算曲率，也没有关系
    fill(cloudCurvature, cloudCurvature + cloudSize, 1000);     
    cloudState = (int*)malloc(cloudSize * sizeof(int));
    cloudSortInd = (int*)malloc(cloudSize * sizeof(int));
    for(int idx = neighbor_size; idx < cloudSize - neighbor_size; idx ++)
    {
        cloudSortInd[idx] = idx;
        cloudState[idx] = 0;
    }

    MarkOccludedPoints();
    int* cloudState_preserve = (int*)malloc(cloudSize * sizeof(int));
    memcpy(cloudState_preserve, cloudState, cloudSize * sizeof(int));

    // 计算每个点到原点的距离, 保存成一个数组是为了后面方便计算曲率
    float* cloudDistance;
    cloudDistance = (float*)malloc(cloudSize * sizeof(float));
    for(size_t i = 0; i < cloudSize; i++)
    {
        // cloudDistance[i] = sqrt(cloud_scan->points[i].x * cloud_scan->points[i].x + 
        //                         cloud_scan->points[i].y * cloud_scan->points[i].y + 
        //                         cloud_scan->points[i].z * cloud_scan->points[i].z);
        // float dis1 = cloudDistance[i];
        cloudDistance[i] = range_image(point_idx_to_image[i].first, point_idx_to_image[i].second);
        // LOG(INFO) << dis1 << "   " << cloudDistance[i];
    }
    
    // 计算每个点的弯曲度
    // lego loam 论文版本
    // for(int idx = neighbor_size; idx < cloudSize - neighbor_size; idx ++)
    // {
    //     float diff_depth = 0;
    //     for(int i = 1; i <= neighbor_size; i++)
    //     {
    //         diff_depth += (cloudDistance[idx - i] + cloudDistance[idx + i]);
    //     }
    //     diff_depth -= 2 * neighbor_size * cloudDistance[idx];
    //     // diff_depth /= 2 * neighbor_size;
    //     diff_depth /= cloudDistance[idx];
    //     diff_depth = diff_depth * diff_depth;

    //     cloudCurvature[idx] = abs(diff_depth);
        
    // }
    // 原版（lego loam代码版本）
    for(int idx = neighbor_size; idx < cloudSize - neighbor_size; idx ++)
    {
        float diff_depth = 0;
        for(int i = 1; i <= neighbor_size; i++)
        {
            diff_depth += (cloudDistance[idx - i] + cloudDistance[idx + i]);
        }
        diff_depth -= 2 * neighbor_size * cloudDistance[idx];
        // diff_depth /= 2 * neighbor_size;
        diff_depth = diff_depth * diff_depth;
        cloudCurvature[idx] = abs(diff_depth);  
    }


    // 保存每个点的弯曲度
    // pcl::PointCloud<pcl::PointXYZI> cloud_curvature(*cloud_scan);
    // for(int idx = 0; idx < cloudSize ; idx ++)
    // {
    //     cloud_curvature.points[idx].intensity = cloudCurvature[idx];
    // }
    // pcl::io::savePCDFileASCII("cloud_scan_curvature.pcd", cloud_curvature);
    
    // 对所有点按照曲率排序，这个排序是先把每个scan分成六份，然后在每一份内按照曲率排序
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        // 把一个scan分成连续的6段，每次只遍历其中一段  sp=start point  ep=end point
        // j = 0  => sp = start                     ep = start + 1/6 * length - 1
        // j = 1  => sp = start +  1/6 * length     ep = start + 2/6 * length - 1
        // j = 2  => sp = start +  2/6 * length     ep = start + 3/6 * length - 1
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // 按弯曲度对这一段里的点从小到大排序，排序结果存在cloudSortInd里对应于sp-ep这一段里
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, 
                    [this](int a, int b)->bool{return (cloudCurvature[a]<cloudCurvature[b]);}
                    );
        }
    }

    // 如果要提取两次，那就用当前传递的参数作为宽松版参数，然后计算得到一个严格版参数
    // 为了保证提取的效果，最好还是需要传递的参数宽松一些，能提出更多的特征点
    if(double_extract)
    {
        // curvature 越小， intersect angle越大，选取点就越严格
        Eigen::MatrixXf strict_image;
        ExtractEdgeFeatures(strict_image, max_curvature / 10.0, intersect_angle_threshold * 2);
        // pcl::io::savePCDFileASCII("corner_strict.pcd", cornerLessSharp);
        // VirtualizeRangeImage("image_strict.png", strict_image, 10, 0.5);
        memcpy(cloudState, cloudState_preserve, cloudSize * sizeof(int));
        
        Eigen::MatrixXf loose_image;
        ExtractEdgeFeatures(loose_image, max_curvature, intersect_angle_threshold);
        // pcl::io::savePCDFileASCII("corner_loose.pcd", cornerLessSharp);
        // VirtualizeRangeImage("image_loose.png", loose_image, 10, 0.5);

        CombineEdgeFeatures(strict_image, loose_image);
    }
    else 
    {
        Eigen::MatrixXf loose_image;
        ExtractEdgeFeatures(loose_image, max_curvature, intersect_angle_threshold);
        // pcl::io::savePCDFileASCII("corner_loose.pcd", cornerLessSharp);
        VirtualizeRangeImage("image_loose.png", loose_image, 10, 0.5);
    }
    // 最后提取平面特征
    ExtractPlaneFeatures();

}

void Velodyne::ExtractEdgeFeatures(Eigen::MatrixXf& picked_image, float max_curvature, float intersect_angle_threshold)
{
    int cloudSize = cloud_scan->points.size();
    const int neighbor_size = 5;

    cornerSharp.clear();
    cornerLessSharp.clear();
    vector<size_t> corner_picked;
    // 遍历所有scan
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        // 把一个scan分成连续的6段，每次只遍历其中一段  sp=start point  ep=end point
        // j = 0  => sp = start                     ep = start + 1/6 * length - 1
        // j = 1  => sp = start +  1/6 * length     ep = start + 2/6 * length - 1
        // j = 2  => sp = start +  2/6 * length     ep = start + 3/6 * length - 1
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;
            int largestPickedNum = 0;   // 选取的弯曲度最大的点的数量
            // 从ep开始选，因为cloudSortInd是按弯曲度从小到大排列的，最后一个点弯曲度最大
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]; 
                if(cloudCurvature[ind] > max_curvature)     // 弯曲度太大的也不要，很可能是测量错误的点
                    continue;
                // 计算入射角，也就是入射激光和激光落点所在的局部平面的夹角
                // 详情见 livox loam论文里的公式(4)
                Eigen::Vector3f vec_a(cloud_scan->points[ind].x, cloud_scan->points[ind].y, cloud_scan->points[ind].z);
                Eigen::Vector3f vec_b(cloud_scan->points[ind + neighbor_size].x - cloud_scan->points[ind - neighbor_size].x,
                                    cloud_scan->points[ind + neighbor_size].y - cloud_scan->points[ind - neighbor_size].y,
                                    cloud_scan->points[ind + neighbor_size].z - cloud_scan->points[ind - neighbor_size].z);
                
                float view_angle = acos(abs(vec_a.dot(vec_b)) / ( range_image(point_idx_to_image[ind].first, point_idx_to_image[ind].second) * vec_b.norm()));
                view_angle *= (180.0 / M_PI);   
                if(view_angle < intersect_angle_threshold || view_angle > 180 - intersect_angle_threshold)
                    continue;

                if (cloudState[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {
                    // 弯曲度最大的两个点放入sharp和lessSharp里，剩下的最大的28个放入lessSharp里
                    largestPickedNum++;
                    if (largestPickedNum <= 3)
                    {                        
                        cloudState[ind] = 2;
                        cornerSharp.push_back(cloud_scan->points[ind]);
                        cornerLessSharp.push_back(cloud_scan->points[ind]);
                        corner_picked.push_back(ind);
                    }
                    else if (largestPickedNum <= 30)
                    {                        
                        cloudState[ind] = 1; 
                        cornerLessSharp.push_back(cloud_scan->points[ind]);
                        corner_picked.push_back(ind);
                    }
                    else
                    {
                        break;
                    }

                    cloudState[ind] = INT16_MAX; 
                    // 遍历当前点之后的连续5个点，计算相邻的两个点之间的距离，大于0.22就认为产生了间断，不能被
                    // 选择了，即对应点的 cloudState=0
                    // 0.2236 * 0.2236 = 0.05
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = cloud_scan->points[ind + l].x - cloud_scan->points[ind + l - 1].x;
                        float diffY = cloud_scan->points[ind + l].y - cloud_scan->points[ind + l - 1].y;
                        float diffZ = cloud_scan->points[ind + l].z - cloud_scan->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudState[ind + l] = INT16_MAX;
                    }
                    // 和上面一样，只是遍历之前的5个点
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = cloud_scan->points[ind + l].x - cloud_scan->points[ind + l + 1].x;
                        float diffY = cloud_scan->points[ind + l].y - cloud_scan->points[ind + l + 1].y;
                        float diffZ = cloud_scan->points[ind + l].z - cloud_scan->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudState[ind + l] = INT16_MAX;
                    }
                }
            }
        }

    }

    // LOG(INFO) << "pick " << corner_picked.size() << " points with loose condition";
    picked_image.resize(N_SCANS, horizon_scans);
    picked_image.fill(0);
    for(const size_t& idx : corner_picked)
    {
        const size_t row = point_idx_to_image.find(idx)->second.first;
        const size_t col = point_idx_to_image.find(idx)->second.second;
        picked_image(row, col) = range_image(row, col);
    }
}

void Velodyne::ExtractPlaneFeatures()
{
    int cloudSize = cloud_scan->points.size();
    const int neighbor_size = 5;
    surfFlat.clear();
    surfLessFlat.clear();
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            int smallestPickedNum = 0;      // 选择的弯曲度最小的点的数量
            // 这里是从sp开始选择的
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudState[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    cloudState[ind] = -1; 
                    surfFlat.push_back(cloud_scan->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }
                    cloudState[ind] = INT16_MAX;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = cloud_scan->points[ind + l].x - cloud_scan->points[ind + l - 1].x;
                        float diffY = cloud_scan->points[ind + l].y - cloud_scan->points[ind + l - 1].y;
                        float diffZ = cloud_scan->points[ind + l].z - cloud_scan->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudState[ind + l] = INT16_MAX;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = cloud_scan->points[ind + l].x - cloud_scan->points[ind + l + 1].x;
                        float diffY = cloud_scan->points[ind + l].y - cloud_scan->points[ind + l + 1].y;
                        float diffZ = cloud_scan->points[ind + l].z - cloud_scan->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudState[ind + l] = INT16_MAX;
                    }
                }
            }
            
            for (int k = sp; k <= ep; k++)
            {
                if (cloudState[k] != 1 && cloudState[k] != 2)
                {
                    surfPointsLessFlatScan->push_back(cloud_scan->points[k]);
                }
            }
        }
        pcl::PointCloud<PointType> surfLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfLessFlatScanDS);

        surfLessFlat += surfLessFlatScanDS;
    }
}

void Velodyne::CombineEdgeFeatures(const Eigen::MatrixXf& strict_image, const Eigen::MatrixXf& loose_image)
{
    cornerLessSharp.clear();
    const int h_size = 3;   // 水平方向的邻域半径
    const int v_size = 2;   // 竖直方向的邻域半径
    stack<pair<size_t, size_t>> s;
    vector<pair<size_t, size_t>> segments;
    size_t seg_count = 0;

    // 保存哪些点被访问过，0代表没被访问，1代表访问过
    Eigen::MatrixXi visited(N_SCANS, horizon_scans);
    visited.fill(0);
    // 保存所有被保留下来的边缘点，只是用来可视化的，删去也无妨
    Eigen::MatrixXi corner(N_SCANS, horizon_scans);
    corner.fill(0);
    // BFS算法遍历两张图，得到较为稳定的边缘点，具体方法受Canny算子提取边缘启发
    // 首先默认strict里的点都是边缘点，loose里大部分是边缘点，少部分不是，最终的目标就是提取出最可靠的边缘
    // 1. 从strict里开始，找到第一个点然后遍历它的邻域，这里的邻域既包括strict也包括loose，直到邻域所有的邻域都遍历了，形成了一个
    // 小区域（segment），就可以认为这个segment里的点都是属于同一个边缘的
    // 2. 如果segment里有足够多的点，就认为它确实是一个边缘，保留下来，否则就舍弃
    // 3. 重复步骤1和2直到strict里所有的点都被遍历了
    for(int i = 0; i < strict_image.rows(); i++)
    {
        for(int j = 0; j < strict_image.cols(); j++)
        {
            if(visited(i, j) > 0 || strict_image(i,j) == 0)
                continue;
            s.push(pair<size_t,size_t>(i,j));
            while(!s.empty())
            {
                const size_t row = s.top().first;
                const size_t col = s.top().second;
                if(visited(row, col) > 0)
                {
                    s.pop();
                    continue;
                }
                visited(row, col) = 1;
                segments.emplace_back(s.top());
                s.pop();
                for(int h = -h_size; h <= h_size; h++)
                {
                    for(int v = -v_size; v <= v_size; v++)
                    {
                        const int curr_row = row + v;
                        const int curr_col = col + h;
                        if(curr_row < 0 || curr_row > N_SCANS - 1 || curr_col < 0 || curr_col > horizon_scans - 1)
                            continue;
                        if(strict_image(curr_row, curr_col) > 0 || loose_image(curr_row, curr_col) > 0)
                            s.emplace(pair<size_t, size_t>(size_t(curr_row), size_t(curr_col)));
                    }
                }
            }
            // LOG(INFO) << "segment size: " << segments.size();
            // 如果某个区域里的点足够多，就保留当前区域的点作为最终提取出的点, 这个segment里的点会保存在两个点云里
            // 一个是统一的点云 corner less sharp，另一个则是每个segment独立的点云，edge_segmented
            if(segments.size() > 4)
            {
                pcl::PointCloud<pcl::PointXYZI> seg;
                for(const pair<size_t,size_t>& p : segments)
                {
                    const size_t& point_idx = image_to_point_idx.find(p)->second;
                    pcl::PointXYZI point = cloud_scan->points[point_idx];
                    cornerLessSharp.push_back(point);
                    point.intensity = seg_count;
                    seg.push_back(point);
                    corner(p.first, p.second) = 1;
                }
                edge_segmented.push_back(seg);
                seg_count++;
            }
            segments.clear();
        }
    }
    // VirtualizeRangeImage("image_visited.png",visited.cast<float>(), 1, 0.5);
    // VirtualizeRangeImage("image_corner.png",corner.cast<float>(), 1, 0.5);
}


// 对点云进行分割，分割的算法来自于 LEGO-LOAM
int Velodyne::Segmentation()
{
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> label_image(N_SCANS, horizon_scans);
    label_image.fill(0);

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;
    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;
    queueIndX = new uint16_t[N_SCANS*horizon_scans];
    queueIndY = new uint16_t[N_SCANS*horizon_scans];
    allPushedIndX = new uint16_t[N_SCANS*horizon_scans];
    allPushedIndY = new uint16_t[N_SCANS*horizon_scans];
    // neighbor iterator for segmentaiton process
    vector<pair<int8_t, int8_t> > neighborIterator = {
        pair<int8_t, int8_t>(-1, 0),
        pair<int8_t, int8_t>(0, 1),
        pair<int8_t, int8_t>(0, -1),
        pair<int8_t, int8_t>(1, 0)
    } ;
    float segmentAlphaX = 0.2 / 180.0 * M_PI;
    float segmentAlphaY = 2.0 / 180.0 * M_PI;
    // float segmentTheta = 60.0 / 180.0 * M_PI; // decrese this value may improve accuracy     竖直放置
    float segmentTheta = 20.0 / 180.0 * M_PI;       // 倾斜放置
    int label_count = 1;
    for (size_t row = 0; row < N_SCANS; ++row)
        for (size_t col = 0; col < horizon_scans; ++col)
            if (label_image(row,col) == 0)
            {
                // use std::queue std::vector std::deque will slow the program down greatly
                float d1, d2, alpha, angle;
                int fromIndX, fromIndY, thisIndX, thisIndY; 
                bool lineCountFlag[N_SCANS] = {false};

                queueIndX[0] = row;
                queueIndY[0] = col;
                int queueSize = 1;
                int queueStartInd = 0;
                int queueEndInd = 1;

                allPushedIndX[0] = row;
                allPushedIndY[0] = col;
                int allPushedIndSize = 1;
                
                while(queueSize > 0){
                    // Pop point
                    fromIndX = queueIndX[queueStartInd];
                    fromIndY = queueIndY[queueStartInd];
                    --queueSize;
                    ++queueStartInd;
                    // Mark popped point
                    label_image(fromIndX, fromIndY) = label_count;
                    // Loop through all the neighboring grids of popped grid
                    for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                        // new index
                        thisIndX = fromIndX + (*iter).first;
                        thisIndY = fromIndY + (*iter).second;
                        // index should be within the boundary
                        if (thisIndX < 0 || thisIndX >= N_SCANS)
                            continue;
                        // at range image margin (left or right side)
                        if (thisIndY < 0)
                            thisIndY = horizon_scans - 1;
                        if (thisIndY >= horizon_scans)
                            thisIndY = 0;
                        // prevent infinite loop (caused by put already examined point back)
                        if (label_image(thisIndX, thisIndY) != 0)
                            continue;

                        d1 = std::max(range_image(fromIndX, fromIndY), range_image(thisIndX, thisIndY));
                                    
                        d2 = std::min(range_image(fromIndX, fromIndY), range_image(thisIndX, thisIndY));
                                    

                        if ((*iter).first == 0)
                            alpha = segmentAlphaX;
                        else
                            alpha = segmentAlphaY;

                        angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                        if (angle > segmentTheta){

                            queueIndX[queueEndInd] = thisIndX;
                            queueIndY[queueEndInd] = thisIndY;
                            ++queueSize;
                            ++queueEndInd;

                            label_image(thisIndX, thisIndY) = label_count;
                            lineCountFlag[thisIndX] = true;

                            allPushedIndX[allPushedIndSize] = thisIndX;
                            allPushedIndY[allPushedIndSize] = thisIndY;
                            ++allPushedIndSize;
                        }
                    }
                }

                // check if this segment is valid
                bool feasibleSegment = false;
                if (allPushedIndSize >= 30)
                    feasibleSegment = true;
                else if (allPushedIndSize >= 5){
                    int lineCount = 0;
                    for (size_t i = 0; i < N_SCANS; ++i)
                        if (lineCountFlag[i] == true)
                            ++lineCount;
                    if (lineCount >= 3)
                        feasibleSegment = true;            
                }
                // segment is valid, mark these points
                if (feasibleSegment == true){
                    ++label_count;
                }else{ // segment is invalid, mark these points
                    for (size_t i = 0; i < allPushedIndSize; ++i){
                        label_image(allPushedIndX[i], allPushedIndY[i]) = INT16_MAX;
                    }
                }
            }
    
    // 根据分割结果重新组织点云，除去不可靠点
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    map<size_t, pair<size_t, size_t>> point_idx_to_image_new;
    map<pair<size_t, size_t>, size_t> image_to_point_idx_new;
    size_t count = 0;
    
    for(size_t idx = 0; idx < cloud_scan->size(); idx++)
    {
        if(label_image(point_idx_to_image[idx].first, point_idx_to_image[idx].second) == INT16_MAX)
            continue;
        else 
        {
            point_idx_to_image_new[count] = point_idx_to_image[idx];
            image_to_point_idx_new[point_idx_to_image[idx]] = count;
            count ++;
            laserCloudScans[cloud_scan->points[idx].intensity].push_back(cloud_scan->points[idx]);
        }
        
    }
    cloud_scan->clear();
    point_idx_to_image.swap(point_idx_to_image_new);
    point_idx_to_image_new.clear();
    image_to_point_idx_new.swap(image_to_point_idx);
    image_to_point_idx_new.clear();
    // 每个scan的前5个点和后6个点都不算
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = cloud_scan->size() + 5;
        *cloud_scan += laserCloudScans[i];
        scanEndInd[i] = cloud_scan->size() - 6;
    }

    return 1;
}

void Velodyne::MarkOccludedPoints()
{
    int cloudSize = cloud_scan->points.size();

    for (int i = 5; i < cloudSize - 6; ++i){

        float depth1 = range_image(point_idx_to_image[i].first, point_idx_to_image[i].second);
        float depth2 = range_image(point_idx_to_image[i + 1].first, point_idx_to_image[i + 1].second);
        int columnDiff = std::abs(int( point_idx_to_image[i].second -  point_idx_to_image[i+1].second));

        if (columnDiff < 10){

            if (depth1 - depth2 > 0.3){
                cloudState[i - 5] = INT16_MAX - 1;
                cloudState[i - 4] = INT16_MAX - 1;
                cloudState[i - 3] = INT16_MAX - 1;
                cloudState[i - 2] = INT16_MAX - 1;
                cloudState[i - 1] = INT16_MAX - 1;
                cloudState[i] = INT16_MAX - 1;
            }else if (depth2 - depth1 > 0.3){
                cloudState[i + 1] = INT16_MAX - 1;
                cloudState[i + 2] = INT16_MAX - 1;
                cloudState[i + 3] = INT16_MAX - 1;
                cloudState[i + 4] = INT16_MAX - 1;
                cloudState[i + 5] = INT16_MAX - 1;
                cloudState[i + 6] = INT16_MAX - 1;
            }
        }

        float depth3 = range_image(point_idx_to_image[i - 1].first, point_idx_to_image[i - 1].second);
        float diff1 = std::abs(float(depth3 - depth1));
        float diff2 = std::abs(float(depth2 - depth1));

        if (diff1 > 0.02 * depth1 && diff2 > 0.02 * depth1)
            cloudState[i] = INT16_MAX - 1;
    }
}

bool Velodyne::UndistortCloud(const Eigen::Matrix4d& T_we)
{
    Eigen::Matrix3d R_we = T_we.block<3,3>(0,0);
    Eigen::Vector3d t_we = T_we.block<3,1>(0,3);
    return UndistortCloud(R_we, t_we);
}

bool Velodyne::UndistortCloud(const Eigen::Matrix3d& R_we, const Eigen::Vector3d& t_we)
{
    if(!IsPoseValid())
        return false;
    // 计算从终止（end）到起始（start）的变换
    const Eigen::Matrix3d R_se = R_wl.transpose() * R_we;
    const Eigen::Vector3d t_se = R_wl.transpose() * (t_we - t_wl);
    const Eigen::Quaterniond q_se(R_se);
    if(cloud->empty())
        LoadLidar(name);
    if(!cloud->empty())
    {
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            double ratio = 1.f * i / cloud->points.size();
            Eigen::Quaterniond q_sc = Eigen::Quaterniond::Identity().slerp(ratio, q_se);
            Eigen::Vector3d t_sc = ratio * t_se;
            Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            point = q_sc * point + t_sc;
            cloud->points[i].x = point.x();
            cloud->points[i].y = point.y();
            cloud->points[i].z = point.z();
        }
        cloud_scan->clear();
        cornerLessSharp.clear();
        cornerSharp.clear();
        surfFlat.clear();
        surfLessFlat.clear();
        return true;
    }
    else 
    {
        return false;
    }
}

void Velodyne::Reset()
{
    cloud_scan->clear();
    cloud_scan.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cornerLessSharp.clear();
    cornerSharp.clear();
    surfLessFlat.clear();
    surfFlat.clear();
    edge_segmented.clear();
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
    range_image.resize(0,0);
    point_idx_to_image.clear();
    image_to_point_idx.clear();
    if(cloudCurvature)
    {
        free(cloudCurvature);
        cloudCurvature = NULL;
    }
    if(cloudSortInd)
    {
        free(cloudSortInd);
        cloudSortInd = NULL;
    }
    if(cloudState)
    {
        free(cloudState);
        cloudState = NULL;
    }
}

bool Velodyne::SaveFeatures(string path)
{
    string base_name(name);
    base_name = base_name.substr(0, base_name.rfind('.'));  // aaa/bbb/ccc.pcd -> aaa/bbb/ccc
    base_name = base_name.substr(base_name.rfind('/') + 1); // aaa/bbb/ccc -> ccc
    if(!cornerLessSharp.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_corner_less_sharp.pcd", cornerLessSharp);
    }
    if(!cornerSharp.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_corner_sharp.pcd", cornerSharp);
    }
    if(!surfLessFlat.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_surf_less_flat.pcd", surfLessFlat);
    }
    if(!surfFlat.empty())
    {
        pcl::io::savePCDFile(path + base_name + "_surf_flat.pcd", surfFlat);
    }
    if(!cloud_scan->empty())
    {
        pcl::io::savePCDFile(path + base_name + "_cloud_scan.pcd", *cloud_scan);
    }
    if(!cloud->empty())
    {
        pcl::io::savePCDFile(path + base_name + "_cloud.pcd", *cloud);
    }
    if(!edge_segmented.empty())
    {
        pcl::PointCloud<pcl::PointXYZI> segment;
        for(const pcl::PointCloud<pcl::PointXYZI>& s : edge_segmented)
            segment += s;
        pcl::io::savePCDFileASCII(path + base_name + "_edge_seg.pcd", segment);
    }
    return true;
}

void Velodyne::VirtualizeRangeImage(std::string file_name, const Eigen::MatrixXf& _range_image, 
                                const float max_range, const float min_range)
{
    cv::Mat img_depth = cv::Mat::zeros(_range_image.rows(), _range_image.cols(), CV_8UC3);
    const float range = max_range - min_range;
    for(int i = 0; i < img_depth.rows; i++)
        for(int j = 0; j < img_depth.cols; j++)
        {
            float real_depth = _range_image(i,j);
            if(real_depth == 0)
            {
                img_depth.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
                continue;
            }
            if(real_depth > max_range)
                real_depth = max_range;
            if(real_depth < min_range)
                real_depth = min_range;
            uchar relative_depth = static_cast<uchar>((real_depth - min_range) / range * 255.0);
            img_depth.at<cv::Vec3b>(i,j) = Gray2Color(relative_depth);
        }
    cv::imwrite(file_name, img_depth);
}

void Velodyne::Transform2LidarWorld()
{
    if(world)
    {
        cout << "already in world coordinate" << endl;
        return;
    }
    Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
    T_wl.block<3,3>(0,0) = R_wl;
    T_wl.block<3,1>(0,3) = t_wl;

    // 从雷达坐标系变换到雷达的世界坐标系
    if(!cloud_scan->empty())
        pcl::transformPointCloud(*cloud_scan, *cloud_scan, T_wl);
    if(!surfFlat.empty())
        pcl::transformPointCloud(surfFlat, surfFlat, T_wl);
    if(!surfLessFlat.empty())
        pcl::transformPointCloud(surfLessFlat, surfLessFlat, T_wl);
    if(!cornerSharp.empty())
        pcl::transformPointCloud(cornerSharp, cornerSharp, T_wl);
    if(!cornerLessSharp.empty())
        pcl::transformPointCloud(cornerLessSharp, cornerLessSharp, T_wl);
    if(!cloud->empty())
        pcl::transformPointCloud(*cloud, *cloud, T_wl);
    world = true;
}

void Velodyne::Transform2Local()
{
    if(!world)
    {
        LOG(ERROR) << "lidar points in not in world coordinate" << endl;
        return;
    }
    Eigen::Matrix3d R_lw = R_wl.transpose();
    Eigen::Vector3d t_lw = -R_lw * t_wl;
    Eigen::Matrix4d T_lw = Eigen::Matrix4d::Identity();
    T_lw.block<3,3>(0,0) = R_lw;
    T_lw.block<3,1>(0,3) = t_lw;

    // 从雷达世界坐标系变回雷达坐标系
    if(!cloud_scan->empty())
        pcl::transformPointCloud(*cloud_scan, *cloud_scan, T_lw);
    if(!surfFlat.empty())
        pcl::transformPointCloud(surfFlat, surfFlat, T_lw);
    if(!surfLessFlat.empty())
        pcl::transformPointCloud(surfLessFlat, surfLessFlat, T_lw);
    if(!cornerSharp.empty())
        pcl::transformPointCloud(cornerSharp, cornerSharp, T_lw);
    if(!cornerLessSharp.empty())
        pcl::transformPointCloud(cornerLessSharp, cornerLessSharp, T_lw);
    if(!cloud->empty())
        pcl::transformPointCloud(*cloud, *cloud, T_lw);

    world = false;
}

Eigen::Vector3d Velodyne::World2Local(Eigen::Vector3d point_w) // 把单个点从雷达世界坐标系变回雷达坐标系
{
    return R_wl.transpose() * point_w - R_wl.transpose() * t_wl;
} 

void Velodyne::SetName(std::string _name)
{
    name = _name;
}

void Velodyne::SetPose(const Eigen::Matrix3d _R_wl, const Eigen::Vector3d _t_wl)
{
    R_wl = _R_wl;
    t_wl = _t_wl;
}
void Velodyne::SetPose(const Eigen::Matrix4d T_wl)
{
    R_wl = T_wl.block<3,3>(0,0);
    t_wl = T_wl.block<3,1>(0,3);
}
void Velodyne::SetRotation(const Eigen::Matrix3d _R_wl)
{
    R_wl = _R_wl;
}
void Velodyne::SetTranslation(const Eigen::Vector3d _t_wl)
{
    t_wl = _t_wl;
}

const Eigen::Matrix4d Velodyne::GetPose() const
{
    Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();
    T_wl.block<3,3>(0,0) = R_wl;
    T_wl.block<3,1>(0,3) = t_wl;
    return T_wl;
}

const bool Velodyne::IsPoseValid() const
{
    if(!isinf(t_wl(0)) && !isinf(t_wl(1)) && !isinf(t_wl(2)) && !R_wl.isZero())
        return true;
    return false;
}

const bool Velodyne::IsInWorldCoordinate() const
{
    return world;
}


