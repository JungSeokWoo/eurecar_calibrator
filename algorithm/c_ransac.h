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

class C_RANASC
{
public:
    C_RANASC() {}
    ~C_RANASC() {}

    void GetEstPlane(unsigned int _num_of_iteration, // input
                     double _inlier_condition, // input
                     vector<cv::Point3f> _input_points, // input
                     vector<cv::Point3f> *_inlier_points, // output
                     vector<cv::Point3f> *_projected_points, // output
                     vector<double> *_plane_param); // output

private:
};

#endif // C_RANSAC_H
