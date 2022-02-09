/*
 * @Author: Diantao Tu
 * @Date: 2022-02-09 16:01:08
 */
#ifndef _FRAME_H_
#define _FRAME_H_

#include <opencv2/opencv.hpp>
#include <string>

class Frame
{
private:
    cv::Mat img;
public:
    std::string name;
    int id;

    Frame(std::string _name, int _id);
    bool EdgeFilter();
    bool InverseDistanceTransform();
    ~Frame();
};





#endif