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
    // 因为提取特征比较慢，所以弄了个简易进度条来显示进度
    ProcessBar bar(frames.size(), 0.2);
    omp_set_num_threads(11);
    #pragma omp parallel for schedule(dynamic)
    for(Frame& f : frames)
    {
        f.EdgeFilter();
        f.InverseDistanceTransform(120, 3);
        bar.Add();
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

double Calibrate::ComputeJc(const pcl::PointCloud<pcl::PointXYZI>& cloud, Frame& frame, const Eigen::Matrix4f& T_cl)
{
    double Jc = 0;
    const cv::Mat img_edge = frame.GetImageEdge();
    pcl::PointCloud<pcl::PointXYZI> cloud_cam;
    pcl::transformPointCloud(cloud, cloud_cam, T_cl);
    for(const pcl::PointXYZI& p : cloud_cam.points)
    {
        Eigen::Vector3f pt_cam(p.x, p.y, p.z);
        cv::Point2i pt_pixel = frame.Camera2Imagei(pt_cam);
        // 小于0说明不在图像范围内
        if(pt_pixel.x < 0)
            continue;
        Jc += img_edge.at<float>(pt_pixel) * p.intensity;
    }
    return Jc;
}

// 根据Fc计算是正确的外参的可能性，论文中的公式4
// 但是这个公式有个问题，概率最大不是1，而是0.5，此时的Fc=1.002
// 当Fc = 0时，结果为0.437
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
    const int max_iteration = 30;
    final_T_cl = init_T_cl;
    cv::Mat depth_image = ProjectLidar2ImageRGB(*lidars[0].cloud_discontinuity, 
            frames[0].GetImageGray(), frames[0].GetIntrinsic(), init_T_cl, 0, 50);
    cv::imwrite("edge_init.png", depth_image);
    cv::imwrite("cloud_init.png", ProjectLidar2ImageRGB(*lidars[0].cloud, 
            frames[0].GetImageGray(), frames[0].GetIntrinsic(), init_T_cl, 0, 50));
    
    cv::Mat img_edge_gray;
    frames[0].GetImageEdge().convertTo(img_edge_gray, CV_8U);
    cv::imwrite("idt_image.png", img_edge_gray);
    
    // 使用扰动后的外参投影到图像上，验证扰动是正确的，用于debug
    // eigen_vector<Eigen::Matrix4f> perturb = PerturbCalibration(init_T_cl, 0.05, 0.01);
    // for(int i = 0; i < perturb.size(); i++)
    // {
    //     cv::imwrite("cloud_" + int2str(i) + ".png", ProjectLidar2ImageRGB(*lidars[0].cloud, 
    //         frames[0].GetImageGray(), frames[0].GetIntrinsic(), perturb[i], 0, 50));
    // }
    // return false;
    
    // 经过扰动后会有729个外参，每个外参在每张图像上都有一个Jc，因此一共会有 729 * window_size 个外参
    // 当窗口向下移动一帧后，只有新加入的那一帧需要计算自己对应的Jc，窗口内其他帧的Jc都已经在上一次计算完毕了，直接使用就行了
    // 为了符合这种操作流程，Jc使用一个矩阵来存储，矩阵尺寸为 window size x 729
    cv::Mat Jc_all = cv::Mat::zeros(window_size, 729, CV_64F);
    for(int idx = 0; idx <= frames.size() - window_size; idx++)
    {
        LOG(INFO) << "window : " << idx;
        // 为了让结果复用，每次开始新的窗口后，就要把上一个窗口的后n-1个结果复制到当前结果的前n-1行
        if(window_size > 1)
            Jc_all.rowRange(1, window_size).copyTo(Jc_all.rowRange(0, window_size - 1));
        int iter = 0;
        double probability = 0;     // 当前外参是正确外参的可能性
        double Fc = 0;
        while(iter < max_iteration )
        {
            // 对外参进行扰动，扰动的结果中，第一个是没有经过扰动的
            eigen_vector<Eigen::Matrix4f> perturb = PerturbCalibration(final_T_cl, 0.1, 0.01);
            // start frame用来判断当前是否需要对窗口内所有的frame计算Jc，只有在两种情况下才需要全部计算
            // 1. 当前迭代不是窗口内的第一次迭代，也就是 iter > 0
            // 2. 现在是整个程序第一次运行，也就是 idx=0
            int start_frame = 0;
            if(idx == 0 || iter > 0)
                start_frame = 0;
            else 
                start_frame = window_size - 1;
            // 计算窗口内图像对应的Jc
            for(int i = start_frame; i < window_size; i++)
            {
                const cv::Mat img_edge = frames[i + idx].GetImageEdge();
                #pragma omp parallel for
                for(int j = 0; j < perturb.size(); j++)
                {
                    double Jc = ComputeJc(*(lidars[i + idx].cloud_discontinuity), frames[i + idx], perturb[j]);
                    #pragma omp critical
                    {
                        Jc_all.at<double>(i, j) = Jc;
                    }
                }
            }
            cv::Mat Jc_each_perturb = cv::Mat::zeros(1, 729, CV_64F);
            // 把矩阵按列求和，得到扰动后的各个外参的Jc
            cv::reduce(Jc_all, Jc_each_perturb, 0, CV_REDUCE_SUM);
            int larger = 0, smaller = 0;
            int largest_idx = 0;
            double largest_Jc = 0;
            for(int j = 0; j < Jc_each_perturb.cols; j++)
            {
                if(Jc_each_perturb.at<double>(0, j) > Jc_each_perturb.at<double>(0,0))
                    larger ++;
                else if(Jc_each_perturb.at<double>(0, j) < Jc_each_perturb.at<double>(0,0))
                    smaller ++;
                if(Jc_each_perturb.at<double>(0, j) > largest_Jc)
                {
                    largest_Jc = Jc_each_perturb.at<double>(0, j);
                    largest_idx = j;
                }
            }
            // Fc 是扰动后Jc变小的数目除以总的扰动数量
            Fc = smaller / 728.0;
            // probability = CorrectProbability(Fc);
            probability = Fc;
            LOG(INFO) << "iter: " << iter << "  probability: " << probability << " larger: " << larger << " smaller: " << smaller;
            iter++;
            if(probability >= 0.9)
                break;
            // 如果到了最后一次迭代，还没有找到当前的最佳结果，那也不更新外参了，把结果交给下一个窗口去找
            if(iter != max_iteration)
                final_T_cl = perturb[largest_idx];
        }
    }

    cv::imwrite("edge_final.png", ProjectLidar2ImageRGB(*lidars[0].cloud_discontinuity, 
            frames[0].GetImageGray(), frames[0].GetIntrinsic(), final_T_cl, 0, 50));
    cv::imwrite("cloud_final.png", ProjectLidar2ImageRGB(*lidars[0].cloud, 
            frames[0].GetImageGray(), frames[0].GetIntrinsic(), final_T_cl, 0, 50));

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
        // cv::imwrite("./" + int2str(i) + ".png", DepthImageRGB(frames[i].GetImageEdge(), 255, 0));
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