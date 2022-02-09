/*
 * @Author: Diantao Tu
 * @Date: 2021-10-24 15:25:01
 */

#include "Virtualization.h"
#include <opencv2/core/eigen.hpp>

using namespace std;

//灰度图转为彩虹图:灰度值255~0分别对应：红、橙、黄、绿、青、蓝。
cv::Vec3b Gray2Color(uchar gray)
{
    cv::Vec3b pixel;
    // if (gray == 0)
    // {
    //     pixel[0] = 0;
    //     pixel[1] = 0;
    //     pixel[2] = 0;
    // }
     if (gray <= 51)
    {
        pixel[0] = 255;
        pixel[1] = gray * 5;
        pixel[2] = 0;
    }
    else if (gray <= 102)
    {
        gray -= 51;
        pixel[0] = 255 - gray * 5;
        pixel[1] = 255;
        pixel[2] = 0;
    }
    else if (gray <= 153)
    {
        gray -= 102;
        pixel[0] = 0;
        pixel[1] = 255;
        pixel[2] = gray * 5;
    }
    else if (gray <= 204)
    {
        gray -= 153;
        pixel[0] = 0;
        pixel[1] = 255 - static_cast<unsigned char>(128.0 * gray / 51.0 + 0.5);
        pixel[2] = 255;
    }
    else
    {
        gray -= 204;
        pixel[0] = 0;
        pixel[1] = 127 - static_cast<unsigned char>(127.0 * gray / 51.0 + 0.5);
        pixel[2] = 255;
    }
    return pixel;
}



void DrawLine(cv::Mat& img, cv::Vec4f line, cv::Scalar color, int thickness, bool panoramic )
{
    if(!panoramic)
    {
        cv::line(img, cv::Point2f(line[0], line[1]), cv::Point2f(line[2], line[3]), color, thickness);
        return;
    }

    Equirectangular eq(img.rows, img.cols);
    vector<cv::Point2f> segments = eq.BreakToSegments(line, 100);
    for(int i = 0; i < segments.size() - 1; i++)
    {
        if(abs(segments[i].x - segments[i+1].x) > 0.8 * img.cols)       
            continue;
        cv::line(img, segments[i], segments[i+1], color, thickness);
    }
    return;
}

void DrawEachLine(const string path, const cv::Mat img, const vector<cv::Vec4f> lines, 
                const cv::Scalar color, const int thickness,  const bool panoramic)
{
    cv::Mat img_line_raw;
    if(img.channels() == 1)
    {
        vector<cv::Mat> images = {img, img, img};
        cv::merge(images, img_line_raw);
    }
    else 
        img_line_raw = img.clone();

    int img_count = 0;
    
    for(size_t i = 0; i < lines.size(); i++)
    {
        cv::Mat img_line = img_line_raw.clone();
        DrawLine(img_line, lines[i], color, thickness, panoramic);
        cv::imwrite(path + "/img_line_" + int2str(img_count) + ".jpg", img_line);
        img_count++;
    }
    return;
}

cv::Mat DrawLinesOnImage(const cv::Mat img, const vector<cv::Vec4f> lines, const vector<cv::Scalar> colors, 
                        const int thickness, const bool panoramic)
{
    cv::Mat img_line;
    if(img.channels() == 1)
    {
        vector<cv::Mat> images = {img, img, img};
        cv::merge(images, img_line);
    }
    else 
        img_line = img.clone();
    
    for(size_t i = 0; i < lines.size(); i++)
        DrawLine(img_line, lines[i], colors[i % colors.size()], thickness, panoramic);
    return img_line;
}

cv::Mat DrawLinePairsOnImage(const cv::Mat img_gray, const vector<LinePair> line_pairs, 
                            const Eigen::Matrix4f T_cl, const int thickness)
{
    Equirectangular eq(img_gray.rows, img_gray.cols);
    vector<cv::Vec4f> img_lines, lidar_lines;
    for(LinePair p : line_pairs)
    {
        img_lines.push_back(p.image_line);
        cv::Point2f p1_pixel = eq.SphereToImage(eq.CamToSphere(TranslatePoint(p.lidar_line_start, T_cl)));
        cv::Point2f p2_pixel = eq.SphereToImage(eq.CamToSphere(TranslatePoint(p.lidar_line_end, T_cl)));
        lidar_lines.push_back(cv::Vec4f(p1_pixel.x, p1_pixel.y, p2_pixel.x, p2_pixel.y));
    }
    cv::Mat img_lidar_line;
    img_lidar_line = DrawLinesOnImage(img_gray, img_lines, vector<cv::Scalar>(1, cv::Scalar(0,0,255)), thickness, true);
    img_lidar_line = DrawLinesOnImage(img_lidar_line, lidar_lines, vector<cv::Scalar>(1, cv::Scalar(255,0,0)), thickness, true);

    return img_lidar_line;
}

cv::Mat DepthImageRGB(const cv::Mat& depth_map, const float max_depth, const float min_depth)
{
    cv::Mat depth_rgb = cv::Mat::zeros(depth_map.size(), CV_8UC3);
    cv::Mat depth_raw;
    if(depth_map.type() ==  CV_16U)
    {
        depth_map.convertTo(depth_raw, CV_32F);
        depth_raw /= 256.0;
    }
    else if(depth_map.type() == CV_32F)
        depth_raw = depth_map.clone();
    else 
    {
        cout << "only support CV_32F or CV_16U" << endl;
        return cv::Mat();
    }
    double max_d, min_d;
    if(max_depth < 0 && min_depth < 0)
    {
        cv::minMaxLoc(depth_raw, new double, &max_d, new cv::Point2i(), new cv::Point2i());
        min_d  = 0;
    }
    else 
    {
        max_d = max_depth;
        min_d = min_depth;
    }
    float depth_range = static_cast<float>(max_d - min_d);
    for(size_t i = 0; i < depth_map.rows; i++)
    {
        for(size_t j = 0; j < depth_map.cols; j++)
        {
            float real_depth = depth_raw.at<float>(i,j);
            if(real_depth == 0)
                depth_rgb.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
            else if(real_depth > max_d)
                depth_rgb.at<cv::Vec3b>(i,j) = Gray2Color(255);
            else if(real_depth < min_d)
                depth_rgb.at<cv::Vec3b>(i,j) = Gray2Color(0);
            else 
                depth_rgb.at<cv::Vec3b>(i,j) = Gray2Color(static_cast<uchar>((real_depth - min_d) / depth_range * 255));
        }
    }
    return depth_rgb;
}

void SaveDepthImageRaw(const cv::Mat& depth_image, const string file_path)
{
    cv::Mat depth_16;
    if(depth_image.type() ==  CV_16U)
    {
        depth_16 = depth_image.clone();
    }
    else if(depth_image.type() == CV_32F)
    {
        cv::Mat tmp = depth_image * 256.0;
        tmp.convertTo(depth_16, CV_16U);
    }
    else 
    {
        cout << "only support CV_32F or CV_16U" << endl;
        return ;
    }
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0); // 无压缩png
    cv::imwrite(file_path, depth_16, compression_params);
    return ;
}

bool CameraCenterPCD(const string& file_name,
                    const eigen_vector<Eigen::Vector3d>& t_wc_list)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_center;
    for(size_t i = 0; i < t_wc_list.size(); i++)
    {
        pcl::PointXYZI point;
        point.x = t_wc_list[i].x();
        point.y = t_wc_list[i].y();
        point.z = t_wc_list[i].z();
        point.intensity = i;
        if(isinf(point.x) || isinf(point.y) || isinf(point.z))
            continue;
        cloud_center.push_back(point);
    }
    pcl::io::savePCDFileASCII(file_name, cloud_center);
    return true;
}


cv::Mat CombineDepthWithRGB(const cv::Mat& depth_image, const cv::Mat& rgb_image, const float max_depth, const float min_depth)
{
    assert(depth_image.rows == rgb_image.rows && depth_image.cols == rgb_image.cols);
    cv::Mat depth_color = DepthImageRGB(depth_image, max_depth, min_depth);
    cv::Mat combined = rgb_image.clone();
    for(int i = 0; i < depth_image.rows; i++)
        for(int j = 0; j < depth_image.cols; j++)
        {
            if(depth_color.at<cv::Vec3b>(i,j) != cv::Vec3b::zeros())
                combined.at<cv::Vec3b>(i,j) = depth_color.at<cv::Vec3b>(i,j);
        }
    return combined;
}

cv::Mat DrawMatchesVertical(const cv::Mat& img1, const std::vector<cv::KeyPoint> keypoints1,
                            const cv::Mat& img2, const std::vector<cv::KeyPoint> keypoints2,
                            const std::vector<cv::DMatch>& matches)
{
    assert(img1.type() == img2.type());
    cv::Mat out = cv::Mat::zeros(img1.rows + img2.rows, max(img1.cols, img2.cols), CV_8UC3);
    img1.copyTo(out.rowRange(0, img1.rows).colRange(0, img1.cols));
    img2.copyTo(out.rowRange(img1.rows, img1.rows + img2.rows).colRange(0, img2.cols));
    cv::Point2f offset(0, img1.rows);
    for(const cv::DMatch& match : matches)
    {
        int idx1 = match.queryIdx;
        int idx2 = match.trainIdx;
        cv::Vec3b color = Gray2Color(static_cast<uchar>(idx1 % 256));
        cv::circle(out, keypoints1[idx1].pt, 10, color, 3);
        cv::circle(out, keypoints2[idx2].pt + offset, 10, color, 3);
        cv::line(out, keypoints1[idx1].pt, keypoints2[idx2].pt + offset, color, 3);
    }
    return out;
}

// 这个函数以后要重写，现在没时间，先凑合着用
bool CameraPoseVisualize(const string& plyfile, const eigen_vector<Eigen::Matrix3d>& R_wc_list, 
                        const eigen_vector<Eigen::Vector3d>& t_wc_list)
{
    if(R_wc_list.size() == 0){
        LOG(ERROR) << "no camera rotation" << endl;
        return false;
    }
    vector<cv::Matx33f> R_cw_list;
    vector<cv::Point3f> t_cw_list;
    for(size_t i = 0; i < R_wc_list.size(); i++)
    {
        const Eigen::Matrix3d& R_wc = R_wc_list[i];
        const Eigen::Vector3d& t_wc = t_wc_list[i];
        // 只导出有绝对位姿的数据
        if(R_wc.isZero())
            continue;
        if(isinf(t_wc.x()) || isinf(t_wc.y()) || isinf(t_wc.z()))
            continue;
        cv::Matx33f R_cw = cv::Matx33f(R_wc(0,0), R_wc(0,1), R_wc(0,2),
                            R_wc(1,0), R_wc(1,1), R_wc(1,2),
                            R_wc(2,0), R_wc(2,1), R_wc(2,2));
        R_cw = R_cw.t();
        R_cw_list.push_back(R_cw);

        cv::Point3f t_cw(t_wc(0), t_wc(1), t_wc(2));
        t_cw = -R_cw * t_cw;
        t_cw_list.push_back(t_cw);
    }
    std::vector<vector<cv::Point3f>> cameras(R_cw_list.size());
    float size = 0.05;
    cv::Matx33f R;
    cv::Point3f T;
    //#pragma omp parallel for
    for(int i = 0; i < R_cw_list.size(); i++)
    { 
        R = R_cw_list[i];      // R_cw
        T = t_cw_list[i];      // t_cw
        if(T.x < -500 || T.x > 500){
            T.x = 0;
            T.y = 0;
            T.z = 0;
        }
        if(T.y < -500 || T.y > 500){
            T.x = 0;
            T.y = 0;
            T.z = 0;
        }
        if(T.z < -500 || T.z > 500){
            T.x = 0;
            T.y = 0;
            T.z = 0;
        }

        cv::Point3f C =  -R.t() * T;        // t_wc = - R_cw.t() * t_cw
        cameras[i].push_back(C);
        cv::Point3f T1;
        cv::Point3f C1;
        T1.x = T.x + size;
        T1.y = T.y + size;
        T1.z = T.z - 2.0 * size;
        C = -R.t() * T1;
        cameras[i].push_back(C);

        T1.x = T.x + -size;
        T1.y = T.y + size;
        T1.z = T.z - 2.0 * size;
        C = -R.t() * T1;
        cameras[i].push_back(C);

        T1.x = T.x + size;
        T1.y = T.y + -size;
        T1.z = T.z - 2.0 * size;
        C = -R.t() * T1;
        cameras[i].push_back(C);

        T1.x = T.x + -size;
        T1.y = T.y + -size;
        T1.z = T.z - 2.0 * size;
        C = -R.t() * T1;
        cameras[i].push_back(C);

    }

    FILE *fp;
    fp = fopen(plyfile.c_str(), "w");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %ld\n", R_cw_list.size() * cameras[0].size());
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "element edge %ld\n", R_cw_list.size() * 8);
    fprintf(fp, "property int vertex1\n");
    fprintf(fp, "property int vertex2\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar gree\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");
    for(int i = 0; i < R_cw_list.size(); i++)
    {
        for (int j = 0; j < cameras[i].size(); j++)
        {
            fprintf(fp, "%f %f %f\n",cameras[i][j].x,cameras[i][j].y,cameras[i][j].z);
        }
    }
    for(int i = 0; i < R_cw_list.size(); i++)
    {
        cv::Vec3b bgr = Gray2Color(static_cast<uchar>(i % 256));
        uchar b = bgr[0];
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size(), i*cameras[i].size() + 1, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size(), i*cameras[i].size() + 2, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size(), i*cameras[i].size() + 3, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size(), i*cameras[i].size() + 4, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size() + 1, i*cameras[i].size() + 2, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size() + 2, i*cameras[i].size() + 4, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size() + 4, i*cameras[i].size() + 3, bgr[0], bgr[1], bgr[2]);
        fprintf(fp, "%ld %ld %d %d %d\n",i*cameras[i].size() + 3, i*cameras[i].size() + 1, bgr[0], bgr[1], bgr[2]);
    }
    fclose(fp);
    return 1;
}