#include "c_camcalib.h"

int CCAMCALIB::AddChessboardPoints(const vector<string> _filelist, cv::Size _board_size)
{
    // the points on the chessboard
    vector<Point2f> image_corners;
    vector<Point3f> object_corners;

    // 3D Scene points
    // Initialize the chessboard corners in the chessboard reference frame
    // The corners are at 3D location (X,Y,Z) = (i,j,0)
    for (int i = 0; i < _board_size.height;i++)
    {
        for (int j = 0; j < _board_size.width;j++)
        {
            object_corners.push_back(Point3f(i,j,0.0f));
        }
    }

    // 2D image points
    Mat image;
    int successes = 0;
    for (int i=0; i < _filelist.size();i++)
    {
        // Open the image
        image = imread(_filelist[i],0);

        // Get the chessboard corners
        bool found = cv::findChessboardCorners(image,_board_size,image_corners);

        if (found)
        {
            //Get subpixel accuracy on the corners
            cv::cornerSubPix(image, image_corners, cv::Size(11,11), cv::Size(-1,-1),
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));

            // If we have a good board, add it to our data
            if (image_corners.size() == _board_size.area())
            {
                AddPoints(image_corners,object_corners);
                successes++;
            }
        }
    }

    cout << "success count : " << successes << endl;

    return successes;
}

void CCAMCALIB::AddPoints(const std::vector<Point2f> &_image_corners, const std::vector<Point3f> &_object_corners)
{
    // 2D image points from one view
    m_image_points.push_back(_image_corners);
    // correspoinding 3D scene points
    m_object_points.push_back(_object_corners);
}

double CCAMCALIB::Calibrate(Size &_image_size)
{
    // undistorter must be reinitialized
    m_must_init_undistort = true;

    // Output rotations and translations
    cv::Matx33d K;
    cv::Vec4d D;
    vector<Mat> rvecs, tvecs;

    int flag;
    double error = calibrateCamera(m_object_points,
                                   m_image_points,
                                   _image_size,
                                   m_camera_matrix,
                                   m_dist_coeffs,
                                   rvecs, tvecs,
                                   CV_CALIB_RATIONAL_MODEL);

//    FileStorage fs("CAM_param.xml",FileStorage::WRITE);
//    fs << "m_camera_matrix" << m_camera_matrix;
//    fs << "m_dist_coeffs" << m_dist_coeffs;
//    fs.release();

    cout << "calibration error : " << error << endl;

    return error;
}

void CCAMCALIB::initUndistortSet(cv::Size _remap_size)
{
    cv::initUndistortRectifyMap(
                m_camera_matrix, // computed camera matrix
                m_dist_coeffs,   // computed distortion matrix
                cv::Mat(),       // optional rectification (none)s
                cv::Mat(),       // camera matrix to generate undistorted
                _remap_size,      // size of undistorted
                CV_32FC1,        // type of output map
                m_map1, m_map2);     //
}

cv::Mat CCAMCALIB::ReMap(const Mat &_image)
{
    cv::Mat undistorted;
    // Apply mapping function
    cv::remap(_image, undistorted, m_map1, m_map2, cv::INTER_LINEAR);

    return undistorted;
}
