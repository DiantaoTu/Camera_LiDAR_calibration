/*
 * @Author: Diantao Tu
 * @Date: 2022-02-10 13:01:38
 */
#include "Calibrate.h"

using namespace std;

Calibrate::Calibrate(const std::vector<Frame>& _frames, const std::vector<Velodyne>& _lidars, const int _window_size):
    frames(_frames), lidars(_lidars), window_size(_window_size)
{
    init_T_cl = Eigen::Matrix4f::Identity();
    final_T_cl = Eigen::Matrix4f::Identity();
}

bool Calibrate::ExtractImageFeatures()
{
    LOG(INFO) << "=============== Extract image features ===============";
    if(frames.empty())
    {
        LOG(ERROR) << "Frames are empty";
        return false;
    }
    omp_set_num_threads(11);
    #pragma omp parallel for schedule(dynamic)
    for(Frame& f : frames)
    {
        f.EdgeFilter();
        f.InverseDistanceTransform(120, 3);
    }
    return true;
}

bool Calibrate::ExtractLidarFeatures()
{
    LOG(INFO) << "================= Extract lidar features ================";
    if(lidars.empty())
    {
        LOG(ERROR) << "lidars are empty";
        return false;
    }
    omp_set_num_threads(11);
    #pragma omp parallel for schedule(dynamic)
    for(int i = 0; i < lidars.size(); i++)
    {
        if(lidars[i].cloud->empty())
            lidars[i].LoadLidar(lidars[i].name);
        lidars[i].ReOrderVLP();
        lidars[i].ExtractFeatures();

    }
    return true;
}


eigen_vector<Eigen::Matrix4f> Calibrate::PerturbCalibration(const Eigen::Matrix4f& T_cl,
                const float rotation_step, const float translation_step)
{
    // 这里为了方便理解，使用了两个vector来保存经过扰动的外参，分别称为perturbed1和perturbed2，这两个是轮流用的
    // 也就是1扰动得到2，然后2扰动得到1，接着1扰动得到2，如此循环，直到在6个方向上的所有扰动都完成了
    eigen_vector<Eigen::Matrix4f> perturbed1 = {T_cl};
    eigen_vector<Eigen::Matrix4f> perturbed2;
    Eigen::Matrix4f delta_T = Eigen::Matrix4f::Identity();
    
    // 在每个轴上的旋转有三个方向，分别是不转，正向转，反向转，delta rotation就是代表着这三个旋转矩阵
    // 而且第一个矩阵永远是固定的单位阵，因为是不旋转，只有后两个在变
    eigen_vector<Eigen::Matrix3f> delta_rotation(3);
    delta_rotation[0] = Eigen::Matrix3f::Identity();
    // 在X轴上旋转
    delta_rotation[1] = Eigen::Matrix3f(Eigen::AngleAxisf(rotation_step * M_PI / 180.f ,Eigen::Vector3f(1,0,0)));
    delta_rotation[2] = Eigen::Matrix3f(Eigen::AngleAxisf(- rotation_step * M_PI / 180.f ,Eigen::Vector3f(1,0,0)));
    for(int i = 0; i < perturbed1.size(); i++)
    {
        for(const Eigen::Matrix3f& delta_r : delta_rotation)
        {
            delta_T.block<3,3>(0,0) = delta_r;
            perturbed2.push_back(delta_T * perturbed1[i]);
        }
    }
    perturbed1.clear();
    // 在y轴上旋转
    delta_rotation[1] = Eigen::Matrix3f(Eigen::AngleAxisf(rotation_step * M_PI / 180.f ,Eigen::Vector3f(0,1,0)));
    delta_rotation[2] = Eigen::Matrix3f(Eigen::AngleAxisf(- rotation_step * M_PI / 180.f ,Eigen::Vector3f(0,1,0)));
 
    for(int i = 0; i < perturbed2.size(); i++)
    {
        for(const Eigen::Matrix3f& delta_r : delta_rotation)
        {
            delta_T.block<3,3>(0,0) = delta_r;
            perturbed1.push_back(delta_T * perturbed2[i]);
        }
    }
    perturbed2.clear();
    // 在z轴上旋转
    delta_rotation[1] = Eigen::Matrix3f(Eigen::AngleAxisf(rotation_step * M_PI / 180.f ,Eigen::Vector3f(0,0,1)));
    delta_rotation[2] = Eigen::Matrix3f(Eigen::AngleAxisf(- rotation_step * M_PI / 180.f ,Eigen::Vector3f(0,0,1)));
 
    for(int i = 0; i < perturbed1.size(); i++)
    {
        for(const Eigen::Matrix3f& delta_r : delta_rotation)
        {
            delta_T.block<3,3>(0,0) = delta_r;
            perturbed2.push_back(delta_T * perturbed1[i]);
        }
    }
    perturbed1.clear();

    // 同理，每个轴上的平移也只有三个方向，不动，正向，反向
    // 因此 delta_t 的第一个永远是[0,0,0]
    eigen_vector<Eigen::Vector3f> delta_translation(3);
    delta_translation[0] = Eigen::Vector3f::Zero();
    delta_T = Eigen::Matrix4f::Identity();
    // 在x轴上平移
    delta_translation[1] = Eigen::Vector3f(translation_step, 0, 0);
    delta_translation[2] = - delta_translation[1];
 
    for(int i = 0; i < perturbed2.size(); i++)
    {
        for(const Eigen::Vector3f& delta_t : delta_translation)
        {
            delta_T.block<3,1>(0,3) = delta_t;
            perturbed1.push_back(delta_T * perturbed2[i]);
        }
    }
    perturbed2.clear();
    // 在y轴上平移
    delta_translation[1] = Eigen::Vector3f(0, translation_step, 0);
    delta_translation[2] = - delta_translation[1];
 
    for(int i = 0; i < perturbed1.size(); i++)
    {
        for(const Eigen::Vector3f& delta_t : delta_translation)
        {
            delta_T.block<3,1>(0,3) = delta_t;
            perturbed2.push_back(delta_T * perturbed1[i]);
        }
    }
    perturbed1.clear();
    // 在z轴上平移
    delta_translation[1] = Eigen::Vector3f(0, 0, translation_step);
    delta_translation[2] = - delta_translation[1];
 
    for(int i = 0; i < perturbed2.size(); i++)
    {
        for(const Eigen::Vector3f& delta_t : delta_translation)
        {
            delta_T.block<3,1>(0,3) = delta_t;
            perturbed1.push_back(delta_T * perturbed2[i]);
        }
    }
    assert(perturbed1.size() == 729);    // 3^6 = 729
    return perturbed1;
}

double Calibrate::ComputeJc(const pcl::PointCloud<pcl::PointXYZI>& cloud, const Frame& frame, const Eigen::Matrix4f& T_cl)
{
    double Jc = 0;
    const cv::Mat img_edge = frame.GetImageEdge();
    for(const pcl::PointXYZI& p : cloud.points)
    {
        Eigen::Vector3f pt_lidar(p.x, p.y, p.z);
        Eigen::Vector3f pt_cam = (T_cl * pt_lidar.homogeneous()).hnormalized();
        cv::Point2i pt_pixel = frame.Camera2Imagei(pt_cam);
        // 小于0说明不在图像范围内
        if(pt_pixel.x < 0)
            continue;
        Jc += img_edge.at<float>(pt_pixel) * p.intensity;
    }
    return Jc;
}

double Calibrate::CorrectProbability(double Fc)
{
    double mu1 = 0.997;
    double sigma1 = 1.4;
    double mu2 = 0.505;
    double sigma2 = 14;
    double p1 = exp(-0.5 * (Fc - mu1) * (Fc - mu1) / sigma1 / sigma1);
    double p2 = exp(-0.5 * (Fc - mu2) * (Fc - mu2) / sigma2 / sigma2);
    return p1 / (p1 + p2);
}

bool Calibrate::StartCalibration()
{
    double best_Jc = 0;
    // 所有的Jc, 初始化为全0
    vector<double> Jc_list(0, 728);
    
    Eigen::Matrix4f best_calibration = Eigen::Matrix4f::Identity();
    // 对外参进行扰动，扰动的结果中，第一个是没有经过扰动的
    eigen_vector<Eigen::Matrix4f> perturb = PerturbCalibration(init_T_cl, 0.3, 0.02);
    for(int i = 0; i < window_size; i++)
    {
        const cv::Mat img_edge = frames[i].GetImageEdge();
        for(int j = 0; j < perturb.size(); j++)
        {
            Jc_list[j] += ComputeJc(*(lidars[i].cloud_discontinuity), frames[i], perturb[j]);
        }
    }
    double Fc = 0;
    int larger = 0, smaller = 0;
    for(const double& Jc : Jc_list)
    {
        if(Jc > Jc_list[0])
            larger ++;
        else if(Jc < Jc_list[0])
            smaller++;
    }
    // Fc 是扰动后Jc变小的数目除以总的扰动数量
    Fc = smaller / 728.0;
    LOG(INFO) << "probobility: " << CorrectProbability(Fc);

    return true;
}

bool Calibrate::LoadEdgeImage(std::string path)
{
    LOG(INFO) << "Load edge images from " << path;
    vector<string> edge_names;
    IterateFiles(path, edge_names, ".xml");
    sort(edge_names.begin(), edge_names.end(), FileNameComp);
    if(edge_names.empty())
    {
        LOG(ERROR) << "edge images are empty";
        return false;
    }
    if(edge_names.size() != frames.size())
        LOG(WARNING) << "number of edge images != number of frame, edge image : " 
                    << edge_names.size() << "  frame : " << frames.size();
    #pragma omp parallel for
    for(int i = 0; i < min(edge_names.size(), frames.size()); i++)
    {
        frames[i].LoadEdgeImage(edge_names[i]);
    }
    LOG(INFO) << "successfully load " << min(edge_names.size(), frames.size()) << " edge images";
    return true;
}

bool Calibrate::SaveEdgeImage(std::string path)
{
    if(boost::filesystem::exists(path))
        boost::filesystem::remove_all(path);
    boost::filesystem::create_directories(path);
    #pragma omp parallel for
    for(const Frame& f: frames)
    {
        f.SaveEdgeImage(path);
    }
    return true;
}

void Calibrate::SetInitCalibration(const Eigen::Matrix4f T_cl)
{
    init_T_cl = T_cl;
}

const Eigen::Matrix4f Calibrate::GetCalibration() const
{
    return final_T_cl;
}