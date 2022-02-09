/*
 * @Author: Diantao Tu
 * @Date: 2022-02-09 16:09:42
 */
#include "Frame.h"

Frame::Frame(std::string _name, int _id):name(_name),id(_id)
{
    if(!name.empty())
        img = cv::imread(name);
}

Frame::~Frame()
{
}
