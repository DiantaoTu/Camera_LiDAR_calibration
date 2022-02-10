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
