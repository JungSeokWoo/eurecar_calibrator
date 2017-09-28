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

    cv::Point3f sum_pt(0,0,0);
    cv::Point3f correction_pt(0,0,0);

    cv::Point3f mean_pt(0,0,0);

    for(size_t i=0;i<_input_points.size();i++)
    {
        KahanSum(_input_points[i].x,sum_pt.x,correction_pt.x);
        KahanSum(_input_points[i].y,sum_pt.y,correction_pt.y);
        KahanSum(_input_points[i].z,sum_pt.z,correction_pt.z);
    }

    mean_pt.x = sum_pt.x/(double)_input_points.size();
    mean_pt.y = sum_pt.y/(double)_input_points.size();
    mean_pt.z = sum_pt.z/(double)_input_points.size();


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

//        sum_zx += z*x;
//        sum_y2 += y*y;
//        sum_zy += z*y;
//        sum_xy += x*y;
//        sum_x2 += x*x;

        KahanSum(z*x,sum_zx,cor_zx);
        KahanSum(y*y,sum_y2,cor_y2);
        KahanSum(z*y,sum_zy,cor_zy);
        KahanSum(x*y,sum_xy,cor_xy);
        KahanSum(x*x,sum_x2,cor_x2);
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

float C_VEC_CAL::InnerProd3D(cv::Point3f _ele_1_vec, cv::Point3f _ele_2_vec)
{
    float return_val;

    return_val = _ele_1_vec.x*_ele_2_vec.x + _ele_1_vec.y*_ele_2_vec.y + _ele_1_vec.z*_ele_2_vec.z;

    return return_val;
}

void C_VEC_CAL::CrossProd3D(cv::Point3f _ele_1_vec, cv::Point3f _ele_2_vec, cv::Point3f &_output)
{
    float x1 = _ele_1_vec.x;
    float y1 = _ele_1_vec.y;
    float z1 = _ele_1_vec.z;

    float x2 = _ele_2_vec.x;
    float y2 = _ele_2_vec.y;
    float z2 = _ele_2_vec.z;

    _output.x = y1*z2 - y2*z1;
    _output.y = -(x1*z2 - z1*x2);
    _output.z = x1*y2 - x2*y1;
}

float C_VEC_CAL::Norm3D(cv::Point3f _input_vec)
{
    float norm_vec = sqrt(_input_vec.x*_input_vec.x + _input_vec.y*_input_vec.y + _input_vec.z*_input_vec.z);
    return norm_vec;
}

cv::Point3f C_VEC_CAL::Sum3D(cv::Point3f _ele_1_vec,cv::Point3f _ele_2_vec)
{
    cv::Point3f return_vec;

    return_vec.x = _ele_1_vec.x + _ele_2_vec.x;
    return_vec.y = _ele_1_vec.y + _ele_2_vec.y;
    return_vec.z = _ele_1_vec.z + _ele_2_vec.z;

    return return_vec;
}

float C_VEC_CAL::CalBetweenAngle(float _left_length, float _right_length, float _bottom_length)
{
    float between_ang = acos((_left_length*_left_length + _right_length*_right_length - _bottom_length*_bottom_length)/(2*_left_length*_right_length));

    return between_ang;
}

cv::Point3f C_VEC_CAL::ObtainOtherSideTri(cv::Point3f _e, float _theta, cv::Point3f _norm_plane)
{
    arma::mat A(4,3);
    arma::mat t(3,1);
    arma::mat B(4,1);

    A(0,0) = _e.x;
    A(0,1) = _e.y;
    A(0,2) = _e.z;
//    A(0,3) = 0;

    A(1,0) = 0;
    A(1,1) = -_e.z;
    A(1,2) = _e.y;
//    A(1,3) = 0;

    A(2,0) = -_e.z;
    A(2,1) = 0;
    A(2,2) = _e.x;
//    A(2,3) = 0;

    A(3,0) = -_e.y;
    A(3,1) = _e.x;
    A(3,2) = 0;
//    A(3,3) = 0;

    B(0,0) = cos(_theta);
    B(1,0) = _norm_plane.x;
    B(2,0) = -_norm_plane.y;
    B(3,0) = _norm_plane.z;

    cv::Point3f vec_t;

//    t = inv(A)*B;
    t = arma::solve(A,B);
    vec_t.x = t(0,0);
    vec_t.y = t(1,0);
    vec_t.z = t(2,0);

    // Check obtain side is proper
    cv::Point3f left_cross_right;

    CrossProd3D(_e,vec_t,left_cross_right);

    if(left_cross_right.x * _norm_plane.x < 0) // wrong!
    {
        B(0,0) = cos(_theta);
        B(1,0) = -_norm_plane.x;
        B(2,0) = _norm_plane.y;
        B(3,0) = -_norm_plane.z;

//        t = inv(A)*B;
        t = arma::solve(A,B);
        vec_t.x = t(0,0);
        vec_t.y = t(1,0);
        vec_t.z = t(2,0);
    }


    return vec_t;
}

cv::Point3f C_VEC_CAL::RotateByArbitraryAxis(cv::Point3f _e, float _theta, cv::Point3f _norm_plane)
{
    float rotationMatrix[4][4];
    float inputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0};
    float outputMatrix[4][1] = {0.0, 0.0, 0.0, 0.0};


    float u = _norm_plane.x;
    float v = _norm_plane.y;
    float w = _norm_plane.z;

    float L = (u*u + v * v + w * w);

    float u2 = u * u;
    float v2 = v * v;
    float w2 = w * w;

    rotationMatrix[0][0] = (u2 + (v2 + w2) * cos(_theta)) / L;
    rotationMatrix[0][1] = (u * v * (1 - cos(_theta)) - w * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[0][2] = (u * w * (1 - cos(_theta)) + v * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[0][3] = 0.0;

    rotationMatrix[1][0] = (u * v * (1 - cos(_theta)) + w * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[1][1] = (v2 + (u2 + w2) * cos(_theta)) / L;
    rotationMatrix[1][2] = (v * w * (1 - cos(_theta)) - u * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[1][3] = 0.0;

    rotationMatrix[2][0] = (u * w * (1 - cos(_theta)) - v * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[2][1] = (v * w * (1 - cos(_theta)) + u * sqrt(L) * sin(_theta)) / L;
    rotationMatrix[2][2] = (w2 + (u2 + v2) * cos(_theta)) / L;
    rotationMatrix[2][3] = 0.0;

    rotationMatrix[3][0] = 0.0;
    rotationMatrix[3][1] = 0.0;
    rotationMatrix[3][2] = 0.0;
    rotationMatrix[3][3] = 1.0;

    inputMatrix[0][0] = _e.x;
    inputMatrix[1][0] = _e.y;
    inputMatrix[2][0] = _e.z;
    inputMatrix[3][0] = 1.0;

    for(int i = 0; i < 4; i++ ){
        for(int j = 0; j < 1; j++){
            outputMatrix[i][j] = 0;
            for(int k = 0; k < 4; k++){
                outputMatrix[i][j] += rotationMatrix[i][k] * inputMatrix[k][j];
            }
        }
    }

    cv::Point3f return_vec;
    return_vec.x = outputMatrix[0][0];
    return_vec.y = outputMatrix[1][0];
    return_vec.z = outputMatrix[2][0];


    cv::Point3f left_cross_right;
    CrossProd3D(_e,return_vec,left_cross_right);
    if(left_cross_right.x * _norm_plane.x < 0) // wrong!
    {
        _theta = -_theta;
        rotationMatrix[0][0] = (u2 + (v2 + w2) * cos(_theta)) / L;
        rotationMatrix[0][1] = (u * v * (1 - cos(_theta)) - w * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[0][2] = (u * w * (1 - cos(_theta)) + v * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[0][3] = 0.0;

        rotationMatrix[1][0] = (u * v * (1 - cos(_theta)) + w * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[1][1] = (v2 + (u2 + w2) * cos(_theta)) / L;
        rotationMatrix[1][2] = (v * w * (1 - cos(_theta)) - u * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[1][3] = 0.0;

        rotationMatrix[2][0] = (u * w * (1 - cos(_theta)) - v * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[2][1] = (v * w * (1 - cos(_theta)) + u * sqrt(L) * sin(_theta)) / L;
        rotationMatrix[2][2] = (w2 + (u2 + v2) * cos(_theta)) / L;
        rotationMatrix[2][3] = 0.0;

        rotationMatrix[3][0] = 0.0;
        rotationMatrix[3][1] = 0.0;
        rotationMatrix[3][2] = 0.0;
        rotationMatrix[3][3] = 1.0;

        inputMatrix[0][0] = _e.x;
        inputMatrix[1][0] = _e.y;
        inputMatrix[2][0] = _e.z;
        inputMatrix[3][0] = 1.0;

        for(int i = 0; i < 4; i++ ){
            for(int j = 0; j < 1; j++){
                outputMatrix[i][j] = 0;
                for(int k = 0; k < 4; k++){
                    outputMatrix[i][j] += rotationMatrix[i][k] * inputMatrix[k][j];
                }
            }
        }
        return_vec.x = outputMatrix[0][0];
        return_vec.y = outputMatrix[1][0];
        return_vec.z = outputMatrix[2][0];

    }

    return return_vec;
}
