#ifndef CUTIL_FUNCS_H
#define CUTIL_FUNCS_H

#include <vector>

// Qt header
#include <QMainWindow>
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QPixmap>
#include <QGraphicsScene>

// opencv header
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

struct cv_line_element{
    cv::Point start_pt;
    cv::Point end_pt;
};

using namespace std;

QImage Mat2QImage(cv::Mat src);

cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size);
cv::Mat ResizeByConst(cv::Mat _ori_img, cv::Size _output_size, double* _ratio);

#endif
