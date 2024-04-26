#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;


bool img_is_vaild(const cv::Mat image, const int threshold_gray = 230 , const double threshold_percent = 0.95 )// 阈值
{
    // 转换为灰度图像
    Mat gray;
    cv::cvtColor(image, gray, COLOR_BGR2GRAY);
    
    // 统计亮度值大于阈值的像素点
    cv::Mat result = Mat::zeros(image.size(), image.type());
    int cnts = 0;
    for(int i = 0; i < gray.rows; i++)
    {
        for(int j = 0; j < gray.cols; j++)
        {
            if(gray.at<uchar>(i, j) > threshold_gray)
            {
                result.at<Vec3b>(i, j) = image.at<Vec3b>(i, j);
                cnts++;
            }
        }
    }
    cout << " cnts is:" << cnts << "  . total is " << gray.rows * gray.cols << endl;

    double percent = 1.0 * cnts / (1.0 * gray.rows * gray.cols) ;
    std::cout << "vaild percent is :" << 1 - percent << endl;
    // 保存结果图片
    // imwrite("result.jpg", result);

    if(percent > threshold_percent)
        return false;
    return true;    
}

int main()
{
    // 读入RGB图像
    Mat image = imread("1.png");


    img_is_vaild(image, 246);
    img_is_vaild(image, 247);
    img_is_vaild(image, 248);
    img_is_vaild(image, 249);
    img_is_vaild(image, 250);
    img_is_vaild(image, 251);
    
    return 0;
}