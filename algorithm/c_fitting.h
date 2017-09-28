#ifndef C_RANSAC_H
#define C_RANSAC_H

// Qt header
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QMetaType>
#include <QStandardItemModel>
#include <QProcess>

// armadillo
#include <armadillo>

// stl header
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <error.h>
#include <algorithm>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <boost/lexical_cast.hpp>

// opencv header
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// openmp
#include <omp.h>

using namespace std;

class C_FITTING
{
public:
    C_FITTING() {}
    ~C_FITTING() {}

    void GetEstPlane(unsigned int _num_of_iteration, // input
                     double _inlier_condition, // input
                     vector<cv::Point3f> _input_points, // input
                     vector<cv::Point3f> *_inlier_points, // output
                     vector<cv::Point3f> *_projected_points, // output
                     vector<double> *_plane_param); // output

    void Get3DLineFitting(vector<cv::Point3f> _input_points, // input,
                          vector<double> *_line_param,
                          cv::Point3f *_avg_pt); // output

    void KahanSum(float value, float & sum, float & correction)
    {
        float term = value - correction;
        float temp = sum + term;
        correction = (temp - sum) - term;
        sum = temp;
    }



private:
};

class C_VEC_CAL
{
public:
    C_VEC_CAL() {}
    ~C_VEC_CAL() {}

    float InnerProd3D(cv::Point3f _ele_1_vec, cv::Point3f _ele_2_vec);

    void CrossProd3D(cv::Point3f _ele_1_vec, cv::Point3f _ele_2_vec, cv::Point3f& _output);

    float Norm3D(cv::Point3f _input_vec);

    cv::Point3f Sum3D(cv::Point3f _ele_1_vec,cv::Point3f _ele_2_vec);

    float CalBetweenAngle(float _left_length, float _right_length, float _bottom_length);

    cv::Point3f ObtainOtherSideTri(cv::Point3f _e, float _theta, cv::Point3f _norm_plane);

    cv::Point3f RotateByArbitraryAxis(cv::Point3f _e, float _theta, cv::Point3f _norm_plane);

};

#endif // C_RANSAC_H
