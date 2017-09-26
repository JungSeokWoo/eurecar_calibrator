#include "c_fitting.h"

void C_FITTING::GetEstPlane(unsigned int _num_of_iteration, double _inlier_condition, vector<cv::Point3f> _input_points, vector<cv::Point3f> *_inlier_points, vector<cv::Point3f> *_projected_points, vector<double> *_plane_param)
{

}

void C_FITTING::Get3DLineFitting(vector<cv::Point3f> _input_points,
                      vector<double> *_line_param,
                      cv::Point3f *_avg_pt)
{
    // z = Ax + By + C
    float A =0;
    float B =0;
    float C =0;

    cv::Point3f sum_pt;
    cv::Point3f correction_pt;

    cv::Point3f mean_pt;

    for(size_t i=0;i<_input_points.size();i++)
    {
        KahanSum(_input_points[i].x,sum_pt.x,correction_pt.x);
        KahanSum(_input_points[i].y,sum_pt.y,correction_pt.y);
        KahanSum(_input_points[i].z,sum_pt.z,correction_pt.z);
    }

    mean_pt.x = sum_pt.x/_input_points.size();
    mean_pt.y = sum_pt.y/_input_points.size();
    mean_pt.z = sum_pt.z/_input_points.size();


    float sum_zx = 0;
    float cor_zx = 0;

    float sum_y2 = 0;
    float cor_y2 = 0;

    float sum_zy = 0;
    float cor_zy = 0;

    float sum_xy = 0;
    float cor_xy = 0;

    float sum_x2 = 0;
    float cor_x2 = 0;

    for(size_t i=0;i<_input_points.size();i++)
    {
        float x = _input_points.at(i).x - mean_pt.x;
        float y = _input_points.at(i).y - mean_pt.y;
        float z = _input_points.at(i).z - mean_pt.z;

        sum_zx += z*x;
        sum_y2 += y*y;
        sum_zy += z*y;
        sum_xy += x*y;
        sum_x2 += x*x;

//        KahanSum(z*x,sum_zx,cor_zx);
//        KahanSum(y*y,sum_y2,cor_y2);
//        KahanSum(z*y,sum_zy,cor_zy);
//        KahanSum(x*y,sum_xy,cor_xy);
//        KahanSum(x*x,sum_x2,cor_x2);
    }

    float xy_sum_2 = sum_xy*sum_xy;

    A = (sum_zx*sum_y2 - sum_zy*sum_xy)/(sum_x2*sum_y2 - xy_sum_2);
    B = (sum_x2*sum_zx - sum_zx*sum_xy)/(sum_x2*sum_y2 - xy_sum_2);
    C = mean_pt.z - A*mean_pt.x - B*mean_pt.y;

    _line_param->push_back(A);
    _line_param->push_back(B);
    _line_param->push_back(C);

    _avg_pt->x = mean_pt.x;
    _avg_pt->y = mean_pt.y;
    _avg_pt->z = mean_pt.z;

//    arma::mat A(num_of_row,3);
//    arma::mat B(num_of_row,1);
//    arma::mat X(3,1);

//    for(uint i = 0; i < num_of_row;i++)
//    {
//        A(i,0) = _input_points[i].x;
//        A(i,1) = _input_points[i].y;
//        A(i,2) = 1.0;
//        B(i,0) = _input_points[i].z;
//    }

//    X = inv(A.t()*A)*A.t()*B;

//    _line_param->push_back(X(0,0));
//    _line_param->push_back(X(1,0));
//    _line_param->push_back(X(2,0));
}

