#ifndef G_MAIN_WINDOW_H
#define G_MAIN_WINDOW_H

// Qt header
#include <QMainWindow>
#include <QDialog>
#include <QFileDialog>
#include <QImage>
#include <QtGui>
#include <QThread>
#include <QPixmap>
#include <QGraphicsScene>
#include <QVTKWidget.h>

// stl
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <dirent.h>
#include <algorithm>
#include <fstream>
#include <sys/time.h>
#include <ctime>
#include <time.h>
#include <cstdlib>

#include "imgproc/util_funcs.hpp"
#include "imgproc/c_camcalib.h"
#include "3d_view/c_3d_viewer.h"
#include "custom_qt/c_custom_scene.h"
#include "custom_qt/c_custom_scene_ipm.h"
#include "algorithm/c_fitting.h"

namespace Ui {
class G_MAIN_WINDOW;
}

class G_MAIN_WINDOW : public QMainWindow
{
    Q_OBJECT

public:
    explicit G_MAIN_WINDOW(QWidget *parent = 0);
    ~G_MAIN_WINDOW();

private slots:
    void on_pushButton_set_imgfolder_load_clicked();

    void on_horizontalSlider_img_sliderMoved(int position);

    void on_pushButton_next_clicked();

    void on_pushButton_prev_clicked();

    void on_pushButton_clicked();

    void on_pushButton_set_intrinsicparam_save_folder_clicked();

    void on_pushButton_set_excalib_lidar_data_path_load_clicked();

    void on_pushButton_set_excalib_img_path_load_clicked();

    void on_pushButton_gen_bb_clicked();

    void on_radioButton_static_alloc_clicked();

    void on_radioButton_relat_move_clicked();

    void on_pushButton_x_loc_smaller_clicked();

    void on_pushButton_x_loc_larger_clicked();

    void on_pushButton_x_size_larger_clicked();

    void on_pushButton_x_size_smaller_clicked();

    void on_pushButton_y_loc_smaller_clicked();

    void on_pushButton_y_loc_larger_clicked();

    void on_pushButton_y_size_larger_clicked();

    void on_pushButton_y_size_smaller_clicked();

    void on_pushButton_z_loc_smaller_clicked();

    void on_pushButton_z_loc_larger_clicked();

    void on_pushButton_z_size_larger_clicked();

    void on_pushButton_z_size_smaller_clicked();

    void on_pushButton_extract_roi_clicked();

    void on_pushButton_fitting_plane_model_clicked();

    void on_pushButton_projection_inlier_2_plane_clicked();

    void on_horizontalSlider_lidar_data_sliderMoved(int position);

    void on_pushButton_excalib_next_clicked();

    void on_pushButton_excalib_prev_clicked();

    void on_pushButton_extrapolation_clicked();

    void on_pushButton_fitting_triangle_clicked();
    
    void on_pushButton_save_calibration_sample_clicked();

    void on_checkBox_show_filtered_pts_clicked();

    void on_pushButton_move_triangle_clicked();

    void on_pushButton_rotate_triangle_clicked();

    void on_pushButton_load_calibration_samples_clicked();

    void on_pushButton_save_calibration_result_clicked();

    void on_pushButton_load_calibration_result_clicked();

    void on_pushButton_display_result_clicked();

    void on_pushButton_remap_clicked();

    void on_pushButton_load_ipm_src_clicked();

    void on_pushButton_load_single_img_clicked();

    void on_pushButton_load_cam_model_file_clicked();

    void on_pushButton_save_remap_img_clicked();

    void on_pushButton_cal_ipm_clicked();

    void on_pushButton_save_ipm_param_clicked();

    void on_pushButton_save_ipm_img_clicked();

    void on_pushButton_cal_corr_clicked();

    void on_pushButton_load_ipm_param_clicked();

    void on_pushButton_convert_to_ipm_pixel_clicked();

private:
    Ui::G_MAIN_WINDOW *ui;


// thread
private:
    C_T_SCENEUPDATE* c_t_sceneupdate = new C_T_SCENEUPDATE;
    C_T_SCENEUPDATE* c_t_sceneupdate_ipm = new C_T_SCENEUPDATE;

private:
    cv::Mat ori_img;
    cv::Mat disp_img;
    QImage disp_img_q;
    QPixmap disp_img_p;
    QGraphicsScene* disp_img_scene = new QGraphicsScene;
    QMutex mtx_disp;

    cv::Mat remap_img;
    cv::Mat remap_disp_img;
    QImage remap_disp_img_q;
    QPixmap remap_disp_img_p;
    QGraphicsScene* remap_disp_img_scene = new QGraphicsScene;

    cv::Mat ipm_src_img;
    cv::Mat ipm_src_img_disp;
    QImage ipm_src_img_q;
    QPixmap ipm_src_img_p;
    C_CUSTOM_SCENE_IPM *ipm_src_img_scene;
    double ipm_disp_about_src_ratio = 0.0;

    cv::Mat ipm_target_img;
    cv::Mat ipm_target_img_disp;
    QImage ipm_target_img_q;
    QPixmap ipm_target_img_p;
    QGraphicsScene* ipm_target_img_scene = new QGraphicsScene;

    cv::Mat m_ipm_lambda;


    string m_imgfolder_path_load;
    string m_imgfile_path_load;

    vector<string> m_imgfile_path_load_list;

    long m_img_ind = 0;
    long m_img_max = 0;

    // Intrinsic calibration object
    CCAMCALIB m_calib_obj;

    string m_intrinsic_xml_path_save;

    // Extrinsic calibration object
    C_3D_VIEWER* c_3d_viewer_lidar_load = new C_3D_VIEWER;
    C_3D_VIEWER* c_3d_viewer_lidar_filtered = new C_3D_VIEWER;

    string m_lidar_data_path_load_str;
    string m_excalib_img_path_load_str;

    double m_bbox_x_min = 0.0;
    double m_bbox_x_max = 0.0;
    double m_bbox_y_min = 0.0;
    double m_bbox_y_max = 0.0;
    double m_bbox_z_min = 0.0;
    double m_bbox_z_max = 0.0;

    double m_bbox_x_center = 0.5;
    double m_bbox_x_size = 0.5;
    double m_bbox_y_center = 0.5;
    double m_bbox_y_size = 0.5;
    double m_bbox_z_center = 0.5;
    double m_bbox_z_size = 0.5;

    cv::Mat excalib_ori_img;
    cv::Mat excalib_disp_img;
    QImage excalib_disp_img_q;
    QPixmap excalib_disp_img_p;
    C_CUSTOM_SCENE* excalib_disp_img_scene;
    QMutex mtx_sceneupdate;
    cv::Size excalib_disp_img_size;
    double m_excalib_resize_frame_ratio;
    double m_excalib_resize_ratio;
    double m_excalib_resize_ratio_total;

    pcl::ModelCoefficients::Ptr m_plane_coefficients;
    pcl::PointIndices::Ptr m_plane_inliers;

    pcl::ModelCoefficients::Ptr m_left_line_coefficients;
    pcl::PointIndices::Ptr m_left_line_inliers;

    pcl::ModelCoefficients::Ptr m_right_line_coefficients;
    pcl::PointIndices::Ptr m_right_line_inliers;

    PointCloudT::Ptr m_cloud_projection;
    PointCloudT::Ptr m_cloud_left_side;
    PointCloudT::Ptr m_cloud_right_side;
    PointCloudT::Ptr m_cloud_triangle_vertice;

    ulong m_point_ind = 0;
    ulong m_point_max = 0;

    C_FITTING m_fitting_obj;
    C_VEC_CAL m_vec_cal_obj;

    cv::Point3f m_u_vec;
    cv::Point3f m_v_vec;

    // plane norm vector
    cv::Point3f m_plane_norm;


    // Calibratino result matrix
    cv::Mat1f  m_transform_mat;

    // Display calibration result
    cv::Mat disp_calib_result_img;
    QImage disp_calib_result_img_q;
    QPixmap disp_calib_result_img_p;
    QGraphicsScene* disp_calib_result_img_scene = new QGraphicsScene;



private:
    void disp_current_img();
    void disp_current_point(PointCloudT::Ptr _cloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer, QVTKWidget* _qvtkwidget);

    void ResetFilteredCloud();
    void MoveTriangle();
    void RotateTraingle();

public slots:
    void SLOT_C_T_SCENEUPDATE_2_MAIN();
    void SLOT_C_T_SCENEUPDATE_2_MAIN_IPM();
};

#endif // G_MAIN_WINDOW_H
