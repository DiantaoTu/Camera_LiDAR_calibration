/*
 * @Author: Diantao Tu
 * @Date: 2022-02-09 16:09:42
 */
#include "Frame.h"

using namespace std;
Frame::Frame(std::string _name, int _id):name(_name),id(_id)
{
}

bool Frame::EdgeFilter()
{
    cv::Mat img_gray = GetImageGray();
    img_edge = cv::Mat::zeros(img_gray.rows, img_gray.cols, CV_32F);

    for(int row = 0; row < img_gray.rows; row++)
        for(int col = 0; col < img_gray.cols; col++)
        {
            int max_value = 0, min_value = 255;
            int curr_value = img_gray.at<uchar>(row, col);
            for(int i = -1; i <= 1; i++)
            {
                if(row + i >= img_gray.rows || row + i < 0)
                    continue;
                for(int j = -1; j<= 1; j++)
                {
                    if(col + j >= img_gray.cols || col + j < 0)
                        continue;

                    if(img_gray.at<uchar>(row + i, col + j) > max_value)
                        max_value = img_gray.at<uchar>(row + i, col + j);
                    if(img_gray.at<uchar>(row + i, col + j) < min_value)
                        min_value = img_gray.at<uchar>(row + i, col + j);
                }
            }
            img_edge.at<float>(row, col) = max(abs(max_value - curr_value), abs(min_value - curr_value));
        }
    // 可视化edge image，这里用了显示深度图的方法，因为可以把edge image当做深度图，最小深度为0，最大深度为255
    cv::imwrite("./edge_image.png", DepthImageRGB(img_edge, 255, 0));
    return true;
}

// 进行距离变换的时候应该是搜索整张图像上所有的点，计算各个点对当前点的边缘性的增益，选择最大的那个
// 但是由于这是一个指数的形式，距离较远的点对当前点的影响就很小了，几乎不可能选到很远的点，因此为了计算速度就设置了一个
// 窗口，只考虑以当前点为中心的窗口内的点对当前点的边缘性的影响
// 随着窗口增大，idt的效果会变得更好，但是运行时间随之增加，因此采用了可变的窗口大小。也就是说，原本就是边缘的点，那么他的窗口就比较小
// 原本是很平坦的点，他的窗口就比较大
bool Frame::InverseDistanceTransform(const int max_half_window_size, const int min_half_window_size)
{
    cv::Mat img_idt = cv::Mat::zeros(img_edge.rows, img_edge.cols, CV_32F);
    const float alpha = 1.f / 3.f;
    const float gamma = 0.98;
    
    for(int row = 0; row < img_idt.rows; row++)
    {
        for(int col = 0; col < img_idt.cols; col++)
        {
            // 为了避免窗口超出图像边界，需要计算一下窗口的左上角以及右下角，这样遍历的时候就只需要在这
            // 两个点的范围之内就行了, 而且为了计算效率，窗口大小是随着edge而变化的
            float ratio = (1.f - img_edge.at<float>(row, col) / 255.f) * (1.f - img_edge.at<float>(row, col) / 255.f);
            const int half_window_size = min_half_window_size + 
                            ratio * (max_half_window_size - min_half_window_size);
            const vector<int> window_border = GetWindowBorder(half_window_size, row, col);
            float max_edge = 0;
            for(int i = window_border[0]; i <= window_border[2]; i++ )
            {
                for(int j = window_border[1]; j <= window_border[3]; j++)
                {
                    float edginess = img_edge.at<float>(i,j) * pow(gamma, max(abs(i - row), abs(j - col)));
                    if(edginess > max_edge)
                        max_edge = edginess;
                }
            }
            img_idt.at<float>(row, col) = alpha * img_edge.at<float>(row, col) + (1.f - alpha) * max_edge;
        }
    }
    // 可视化 idt 之后的结果
    cv::imwrite("./idt_image.png", DepthImageRGB(img_idt, 255, 0));
    return true;
}

std::vector<int> Frame::GetWindowBorder(const int half_window_size, const int center_row, const int center_col)
{
    int start_row, start_col;   // 左上角的行列
    int end_row, end_col;       // 右下角的行列
    start_row = max(center_row - half_window_size, 0);
    end_row = min(center_row + half_window_size, img_edge.rows - 1);
    start_col = max(center_col - half_window_size, 0);
    end_col = min(center_col + half_window_size, img_edge.cols - 1);
    vector<int> border = {start_row, start_col, end_row, end_col};
    return border;
}

cv::Mat Frame::GetImageColor() const
{
    return cv::imread(name);
}

cv::Mat Frame::GetImageGray() const
{
    return cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
}

const cv::Mat& Frame::GetImageEdge() const 
{
    return img_edge;
}

Frame::~Frame()
{
}
