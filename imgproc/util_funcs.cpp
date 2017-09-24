#include "util_funcs.hpp"

QImage Mat2QImage(cv::Mat src)
{
    cv::Mat temp;
    cv::cvtColor(src,temp,CV_BGR2RGB);
    QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    dest.bits(); // enforce deep copy, see documentation
    // of QImage::QImage ( const uchar * data, int width, int height, Format format )
    return dest;
}

cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size)
{
    cv::Mat output_img;
    double width_ratio = (double)_ori_img.cols/(double)_output_size.width;
    double height_ratio = (double)_ori_img.rows/(double)_output_size.height;

    if(width_ratio > height_ratio) // resize by width ratio
    {
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/width_ratio,(1.0)/width_ratio);
    }
    else
    {
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/height_ratio,(1.0)/height_ratio);
    }

    return output_img;
}


cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size, double* _ratio)
{
    cv::Mat output_img;
    double width_ratio = (double)_ori_img.cols/(double)_output_size.width;
    double height_ratio = (double)_ori_img.rows/(double)_output_size.height;

    if(width_ratio > height_ratio) // resize by width ratio
    {
        *_ratio = (1.0)/width_ratio;
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/width_ratio,(1.0)/width_ratio);
    }
    else
    {
        *_ratio = (1.0)/height_ratio;
        cv::resize(_ori_img,output_img,cv::Size(),(1.0)/height_ratio,(1.0)/height_ratio);
    }

    return output_img;
}
