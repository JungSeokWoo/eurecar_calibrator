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

private:
    Ui::G_MAIN_WINDOW *ui;


// thread
private:
    C_T_SCENEUPDATE* c_t_sceneupdate = new C_T_SCENEUPDATE;

private:
    cv::Mat ori_img;
    cv::Mat disp_img;
    QImage disp_img_q;
    QPixmap disp_img_p;
    QGraphicsScene* disp_img_scene = new QGraphicsScene;
    QMutex mtx_disp;

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
    PointCloudT::Ptr m_cloud_projection;


private:
    void disp_current_img();

public slots:
    void SLOT_C_T_SCENEUPDATE_2_MAIN();
};

#endif // G_MAIN_WINDOW_H
