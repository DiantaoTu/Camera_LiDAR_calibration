/*
 * @Author: Diantao Tu
 * @Date: 2022-02-09 16:01:08
 */
#ifndef _FRAME_H_
#define _FRAME_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <glog/logging.h>
#include "Virtualization.h"

class Frame
{
private:
    cv::Mat img_edge;
    /**
     * @description: 计算以某个点为中心的窗口的左上角的起始点以及右下角的终止点 
     * @param half_window_size {int} 半窗口大小
     * @param center_row {int} 中心点所在行
     * @param center_col {int} 中心点所在列
     * @return {vector<int>} 窗口左上角的行列以及右下角的行列
     */    
    std::vector<int> GetWindowBorder(const int half_window_size, const int center_row, const int center_col);
public:
    std::string name;
    int id;

    Frame(std::string _name, int _id);
    bool EdgeFilter();
    bool InverseDistanceTransform(const int max_half_window_size, const int min_half_window_size);
    cv::Mat GetImageGray() const;
    cv::Mat GetImageColor() const;
    const cv::Mat& GetImageEdge() const;
    ~Frame();
};





#endif