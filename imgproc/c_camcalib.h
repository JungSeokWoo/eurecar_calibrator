#ifndef CCAMCALIB_H
#define CCAMCALIB_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class CCAMCALIB {
public:
    CCAMCALIB() : m_flag(0), m_must_init_undistort(true) {

    }
    ~CCAMCALIB() {}

private:
    // the points in world coordinates
    std::vector<std::vector<cv::Point3f> > m_object_points;

    // the point positions in pixels
    std::vector<std::vector<cv::Point2f> > m_image_points;

    // flag to specify how calibration is done
    int m_flag;

    // used in image undistortion
    cv::Mat m_map1, m_map2;
    bool m_must_init_undistort;

public:

    // output Matrices
    cv::Mat m_camera_matrix;
    cv::Mat m_dist_coeffs;

    int AddChessboardPoints(const vector<string> _filelist, cv::Size _board_size);
    void AddPoints(const std::vector<cv::Point2f>& _image_corners, const std::vector<cv::Point3f>& _object_corners);
    double Calibrate(cv::Size &_image_size);
    cv::Mat ReMap(const cv::Mat& _image);
};


#endif // CCAMCALIB_H
