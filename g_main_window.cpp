#include "g_main_window.h"
#include "ui_g_main_window.h"

G_MAIN_WINDOW::G_MAIN_WINDOW(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::G_MAIN_WINDOW)
{
    ui->setupUi(this);

    excalib_disp_img_size.width = ui->graphicsView_excalib_img_load->geometry().width();
    excalib_disp_img_size.height = ui->graphicsView_excalib_img_load->geometry().height();

    excalib_disp_img_scene = new C_CUSTOM_SCENE(c_t_sceneupdate);

    ipm_src_img_scene = new C_CUSTOM_SCENE_IPM(c_t_sceneupdate_ipm);

    ui->qvtkWidget_lidar_data_load->SetRenderWindow(c_3d_viewer_lidar_load->viewer->getRenderWindow());
    c_3d_viewer_lidar_load->viewer->setupInteractor(ui->qvtkWidget_lidar_data_load->GetInteractor(),ui->qvtkWidget_lidar_data_load->GetRenderWindow());
    c_3d_viewer_lidar_load->viewer->setBackgroundColor(0,0,0);
    c_3d_viewer_lidar_load->viewer->addCoordinateSystem(1.0);
    c_3d_viewer_lidar_load->viewer->addPointCloud(c_3d_viewer_lidar_load->cloud,"cloud");
    c_3d_viewer_lidar_load->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud");

    ui->qvtkWidget_lidar_data_filtered->SetRenderWindow(c_3d_viewer_lidar_filtered->viewer->getRenderWindow());
    c_3d_viewer_lidar_filtered->viewer->setupInteractor(ui->qvtkWidget_lidar_data_filtered->GetInteractor(),ui->qvtkWidget_lidar_data_filtered->GetRenderWindow());
    c_3d_viewer_lidar_filtered->viewer->setBackgroundColor(0,0,0);
    c_3d_viewer_lidar_filtered->viewer->addCoordinateSystem(1.0);
    c_3d_viewer_lidar_filtered->viewer->addPointCloud(c_3d_viewer_lidar_filtered->cloud,"cloud");
    c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud");

    m_cloud_projection.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->addPointCloud(m_cloud_projection,"cloud_proj");
    c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"cloud_proj");

    m_cloud_left_side.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->addPointCloud(m_cloud_left_side,"cloud_left_side");
    c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"cloud_left_side");

    m_cloud_right_side.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->addPointCloud(m_cloud_right_side,"cloud_right_side");
    c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"cloud_right_side");

    m_cloud_triangle_vertice.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->addPointCloud(m_cloud_triangle_vertice,"cloud_triangle_vertice");
    c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,7,"cloud_triangle_vertice");

    connect(c_t_sceneupdate,SIGNAL(SIG_C_T_SCENEUPDATE_2_MAIN()),this,SLOT(SLOT_C_T_SCENEUPDATE_2_MAIN()));
    connect(c_t_sceneupdate_ipm,SIGNAL(SIG_C_T_SCENEUPDATE_2_MAIN_IPM()),this,SLOT(SLOT_C_T_SCENEUPDATE_2_MAIN_IPM()));
}

G_MAIN_WINDOW::~G_MAIN_WINDOW()
{
    delete ui;
}

void G_MAIN_WINDOW::on_pushButton_set_imgfolder_load_clicked()
{
    QString directory;
    directory = QFileDialog::getExistingDirectory(this,"Find Img folder",QDir::currentPath());

    if (!directory.isEmpty())
    {
        ui->lineEdit_imgfolder_load->setText(directory);
        m_imgfolder_path_load = directory.toStdString();

        string dir_file;
        DIR *dir;
        struct dirent *ent;

        m_imgfile_path_load_list.clear();

        dir = opendir(m_imgfolder_path_load.c_str());

        if (dir != NULL)
        {
            while ((ent = readdir(dir)) != NULL)
            {
                dir_file = ent->d_name;
                if((dir_file == ".") || (dir_file == ".." ))
                    continue;

                m_imgfile_path_load = m_imgfolder_path_load + "/" + dir_file;
                m_imgfile_path_load_list.push_back(m_imgfile_path_load);
            }
            closedir(dir);
        }
        std::sort(m_imgfile_path_load_list.begin(),m_imgfile_path_load_list.end());

        m_img_ind = 0;
        m_img_max = m_imgfile_path_load_list.size()-1;
        m_imgfile_path_load = m_imgfile_path_load_list.at(m_img_ind);

        ui->horizontalSlider_img->setMaximum(m_img_max);
        ui->horizontalSlider_img->setValue(m_img_ind);

        disp_current_img();
    }
}

void G_MAIN_WINDOW::disp_current_img()
{

    ui->lineEdit_imgind->setText(QString::number(m_img_ind+1));
    ui->lineEdit_imgtotal->setText(QString::number(m_img_max+1));

    ori_img = cv::imread(m_imgfile_path_load_list.at(m_img_ind));
    if(ori_img.empty())
        return;

    disp_img = ResizeByConst(ori_img,cv::Size(ui->graphicsView_img_load->geometry().width(),ui->graphicsView_img_load->geometry().height()));
    disp_img_q = Mat2QImage(disp_img);
    disp_img_p.convertFromImage(disp_img_q);
    disp_img_scene->clear();
    disp_img_scene->addPixmap(disp_img_p);

    ui->graphicsView_img_load->setScene(disp_img_scene);
    ui->graphicsView_img_load->show();
}


void G_MAIN_WINDOW::on_horizontalSlider_img_sliderMoved(int position)
{
    m_img_ind = ui->horizontalSlider_img->value();

    disp_current_img();
}

void G_MAIN_WINDOW::on_pushButton_next_clicked()
{
    if(m_img_ind + 1 > m_img_max)
    {
        m_img_ind = m_img_max;
    }
    else
    {
        m_img_ind++;
    }

    ui->horizontalSlider_img->setValue(m_img_ind);
    disp_current_img();
}

void G_MAIN_WINDOW::on_pushButton_prev_clicked()
{
    if(m_img_ind -1 < 0)
    {
        m_img_ind = 0;
    }
    else
    {
        m_img_ind--;
    }
    ui->horizontalSlider_img->setValue(m_img_ind);
    disp_current_img();
}

void G_MAIN_WINDOW::on_pushButton_clicked()
{
    m_calib_obj.AddChessboardPoints(m_imgfile_path_load_list,cv::Size(ui->lineEdit_board_width->text().toInt(),ui->lineEdit_board_height->text().toInt()));
    cv::Size img_size = ori_img.size();
    m_calib_obj.Calibrate(img_size);
    string intrinsic_calib_xml_file_path = m_intrinsic_xml_path_save + "/CAM_PARAM.xml";

    FileStorage fs(intrinsic_calib_xml_file_path,FileStorage::WRITE);
    fs << "m_camera_matrix" << m_calib_obj.m_camera_matrix;
    fs << "m_dist_coeffs" << m_calib_obj.m_dist_coeffs;
    fs.release();

}

void G_MAIN_WINDOW::on_pushButton_set_intrinsicparam_save_folder_clicked()
{
    QString directory;
    directory = QFileDialog::getExistingDirectory(this,"Find Img folder",QDir::currentPath());

    if (!directory.isEmpty())
    {
        ui->lineEdit_intrinsic_xml_savepath->setText(directory);
        m_intrinsic_xml_path_save = directory.toStdString();
    }
}

void G_MAIN_WINDOW::on_pushButton_set_excalib_lidar_data_path_load_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select lidar data");

    if (!directory.isEmpty())
    {
        ui->lineEdit_excalib_lidar_data_path_load->setText(directory);
        m_lidar_data_path_load_str = directory.toStdString();

        vector<double> load_pt_data_x;
        vector<double> load_pt_data_y;
        vector<double> load_pt_data_z;
        vector<uint8_t> load_pt_data_r;
        vector<uint8_t> load_pt_data_g;
        vector<uint8_t> load_pt_data_b;

        cv::FileStorage fs(m_lidar_data_path_load_str,cv::FileStorage::READ);
        fs["pt_data_x"] >> load_pt_data_x;
        fs["pt_data_y"] >> load_pt_data_y;
        fs["pt_data_z"] >> load_pt_data_z;
        fs["pt_data_r"] >> load_pt_data_r;
        fs["pt_data_g"] >> load_pt_data_g;
        fs["pt_data_b"] >> load_pt_data_b;
        fs.release();

        c_3d_viewer_lidar_load->cloud->clear();
        for(unsigned int pt_ind = 0;pt_ind < load_pt_data_x.size();pt_ind++)
        {
            pcl::PointXYZRGBA load_pt;

            load_pt.x = load_pt_data_x.at(pt_ind);
            load_pt.y = load_pt_data_y.at(pt_ind);
            load_pt.z = load_pt_data_z.at(pt_ind);

            load_pt.r = load_pt_data_r.at(pt_ind);
            load_pt.g = load_pt_data_g.at(pt_ind);
            load_pt.b = load_pt_data_b.at(pt_ind);

            c_3d_viewer_lidar_load->cloud->points.push_back(load_pt);
        }

        c_3d_viewer_lidar_load->viewer->updatePointCloud(c_3d_viewer_lidar_load->cloud,"cloud");
        ui->qvtkWidget_lidar_data_load->update();
    }
}

void G_MAIN_WINDOW::on_pushButton_set_excalib_img_path_load_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select image");

    if (!directory.isEmpty())
    {
        ui->lineEdit_excalib_img_path_load->setText(directory);
        m_excalib_img_path_load_str = directory.toStdString();

        excalib_ori_img = cv::imread(m_excalib_img_path_load_str);

        m_excalib_resize_frame_ratio = (double)excalib_ori_img.cols/(double)ui->lineEdit_camera_frame_width->text().toDouble();
        excalib_disp_img = ResizeByConst(excalib_ori_img,excalib_disp_img_size,&m_excalib_resize_ratio);
        excalib_disp_img_q = Mat2QImage(excalib_disp_img);
        excalib_disp_img_p.convertFromImage(excalib_disp_img_q);
        excalib_disp_img_scene->clear();
        excalib_disp_img_scene->addPixmap(excalib_disp_img_p);

        ui->graphicsView_excalib_img_load->setScene(excalib_disp_img_scene);
        ui->graphicsView_excalib_img_load->show();

        m_excalib_resize_ratio_total = m_excalib_resize_frame_ratio*m_excalib_resize_ratio;

    }
}

void G_MAIN_WINDOW::on_pushButton_gen_bb_clicked()
{
    if(ui->radioButton_static_alloc->isChecked())
    {

        m_bbox_x_min = ui->lineEdit_x_min->text().toDouble();
        m_bbox_x_max = ui->lineEdit_x_max->text().toDouble();
        m_bbox_y_min = ui->lineEdit_y_min->text().toDouble();
        m_bbox_y_max = ui->lineEdit_y_max->text().toDouble();
        m_bbox_z_min = ui->lineEdit_z_min->text().toDouble();
        m_bbox_z_max = ui->lineEdit_z_max->text().toDouble();

        c_3d_viewer_lidar_load->viewer->removeShape("cube");
        c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
        c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
        ui->qvtkWidget_lidar_data_load->update();
    }
    else
    {
        m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
        m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
        m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
        m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
        m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
        m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

        c_3d_viewer_lidar_load->viewer->removeShape("cube");
        c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
        c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
        ui->qvtkWidget_lidar_data_load->update();
    }
}

void G_MAIN_WINDOW::on_radioButton_static_alloc_clicked()
{
    if(ui->radioButton_static_alloc->isChecked())
    {
        ui->lineEdit_x_max->setEnabled(true);
        ui->lineEdit_x_min->setEnabled(true);
        ui->lineEdit_y_max->setEnabled(true);
        ui->lineEdit_y_min->setEnabled(true);
        ui->lineEdit_z_max->setEnabled(true);
        ui->lineEdit_z_min->setEnabled(true);

        ui->pushButton_x_loc_larger->setEnabled(false);
        ui->pushButton_x_loc_smaller->setEnabled(false);
        ui->pushButton_x_size_larger->setEnabled(false);
        ui->pushButton_x_size_smaller->setEnabled(false);

        ui->pushButton_y_loc_larger->setEnabled(false);
        ui->pushButton_y_loc_smaller->setEnabled(false);
        ui->pushButton_y_size_larger->setEnabled(false);
        ui->pushButton_y_size_smaller->setEnabled(false);

        ui->pushButton_z_loc_larger->setEnabled(false);
        ui->pushButton_z_loc_smaller->setEnabled(false);
        ui->pushButton_z_size_larger->setEnabled(false);
        ui->pushButton_z_size_smaller->setEnabled(false);
    }
    else
    {
        ui->lineEdit_x_max->setEnabled(false);
        ui->lineEdit_x_min->setEnabled(false);
        ui->lineEdit_y_max->setEnabled(false);
        ui->lineEdit_y_min->setEnabled(false);
        ui->lineEdit_z_max->setEnabled(false);
        ui->lineEdit_z_min->setEnabled(false);

        ui->pushButton_x_loc_larger->setEnabled(true);
        ui->pushButton_x_loc_smaller->setEnabled(true);
        ui->pushButton_x_size_larger->setEnabled(true);
        ui->pushButton_x_size_smaller->setEnabled(true);

        ui->pushButton_y_loc_larger->setEnabled(true);
        ui->pushButton_y_loc_smaller->setEnabled(true);
        ui->pushButton_y_size_larger->setEnabled(true);
        ui->pushButton_y_size_smaller->setEnabled(true);

        ui->pushButton_z_loc_larger->setEnabled(true);
        ui->pushButton_z_loc_smaller->setEnabled(true);
        ui->pushButton_z_size_larger->setEnabled(true);
        ui->pushButton_z_size_smaller->setEnabled(true);
    }
}

void G_MAIN_WINDOW::on_radioButton_relat_move_clicked()
{
    if(ui->radioButton_static_alloc->isChecked())
    {
        ui->lineEdit_x_max->setEnabled(true);
        ui->lineEdit_x_min->setEnabled(true);
        ui->lineEdit_y_max->setEnabled(true);
        ui->lineEdit_y_min->setEnabled(true);
        ui->lineEdit_z_max->setEnabled(true);
        ui->lineEdit_z_min->setEnabled(true);

        ui->pushButton_x_loc_larger->setEnabled(false);
        ui->pushButton_x_loc_smaller->setEnabled(false);
        ui->pushButton_x_size_larger->setEnabled(false);
        ui->pushButton_x_size_smaller->setEnabled(false);

        ui->pushButton_y_loc_larger->setEnabled(false);
        ui->pushButton_y_loc_smaller->setEnabled(false);
        ui->pushButton_y_size_larger->setEnabled(false);
        ui->pushButton_y_size_smaller->setEnabled(false);

        ui->pushButton_z_loc_larger->setEnabled(false);
        ui->pushButton_z_loc_smaller->setEnabled(false);
        ui->pushButton_z_size_larger->setEnabled(false);
        ui->pushButton_z_size_smaller->setEnabled(false);
    }
    else
    {
        ui->lineEdit_x_max->setEnabled(false);
        ui->lineEdit_x_min->setEnabled(false);
        ui->lineEdit_y_max->setEnabled(false);
        ui->lineEdit_y_min->setEnabled(false);
        ui->lineEdit_z_max->setEnabled(false);
        ui->lineEdit_z_min->setEnabled(false);

        ui->pushButton_x_loc_larger->setEnabled(true);
        ui->pushButton_x_loc_smaller->setEnabled(true);
        ui->pushButton_x_size_larger->setEnabled(true);
        ui->pushButton_x_size_smaller->setEnabled(true);

        ui->pushButton_y_loc_larger->setEnabled(true);
        ui->pushButton_y_loc_smaller->setEnabled(true);
        ui->pushButton_y_size_larger->setEnabled(true);
        ui->pushButton_y_size_smaller->setEnabled(true);

        ui->pushButton_z_loc_larger->setEnabled(true);
        ui->pushButton_z_loc_smaller->setEnabled(true);
        ui->pushButton_z_size_larger->setEnabled(true);
        ui->pushButton_z_size_smaller->setEnabled(true);
    }
}

void G_MAIN_WINDOW::on_pushButton_x_loc_smaller_clicked()
{
    m_bbox_x_center -= 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_x_loc_larger_clicked()
{
    m_bbox_x_center += 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_x_size_larger_clicked()
{
    m_bbox_x_size += 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_x_size_smaller_clicked()
{
    m_bbox_x_size -= 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_y_loc_smaller_clicked()
{
    m_bbox_y_center -= 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_y_loc_larger_clicked()
{
    m_bbox_y_center += 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_y_size_larger_clicked()
{
    m_bbox_y_size += 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_y_size_smaller_clicked()
{
    m_bbox_y_size -= 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_z_loc_smaller_clicked()
{
    m_bbox_z_center -= 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_z_loc_larger_clicked()
{
    m_bbox_z_center += 0.2;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_z_size_larger_clicked()
{
    m_bbox_z_size += 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_z_size_smaller_clicked()
{
    m_bbox_z_size -= 0.1;

    m_bbox_x_min = m_bbox_x_center - m_bbox_x_size;
    m_bbox_x_max = m_bbox_x_center + m_bbox_x_size;
    m_bbox_y_min = m_bbox_y_center - m_bbox_y_size;
    m_bbox_y_max = m_bbox_y_center + m_bbox_y_size;
    m_bbox_z_min = m_bbox_z_center - m_bbox_z_size;
    m_bbox_z_max = m_bbox_z_center + m_bbox_z_size;

    c_3d_viewer_lidar_load->viewer->removeShape("cube");
    c_3d_viewer_lidar_load->viewer->addCube(m_bbox_x_min,m_bbox_x_max,m_bbox_y_min,m_bbox_y_max,m_bbox_z_min,m_bbox_z_max,0.5,0.5,0.5,"cube",0);
    c_3d_viewer_lidar_load->viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cube");
    ui->qvtkWidget_lidar_data_load->update();
}

void G_MAIN_WINDOW::on_pushButton_extract_roi_clicked()
{
    ResetFilteredCloud();

    c_3d_viewer_lidar_filtered->cloud.reset(new PointCloudT);

    for(unsigned int ind = 0; ind < c_3d_viewer_lidar_load->cloud->points.size();ind++)
    {
        double pt_x = c_3d_viewer_lidar_load->cloud->points.at(ind).x;
        double pt_y = c_3d_viewer_lidar_load->cloud->points.at(ind).y;
        double pt_z = c_3d_viewer_lidar_load->cloud->points.at(ind).z;
        uint8_t pt_r = c_3d_viewer_lidar_load->cloud->points.at(ind).r;
        uint8_t pt_g = c_3d_viewer_lidar_load->cloud->points.at(ind).g;
        uint8_t pt_b = c_3d_viewer_lidar_load->cloud->points.at(ind).b;

        if((m_bbox_x_min <= pt_x) && (m_bbox_x_max > pt_x) && (m_bbox_y_min <= pt_y) && (m_bbox_y_max > pt_y) && (m_bbox_z_min <= pt_z) &&  (m_bbox_z_max > pt_z))
        {
            pcl::PointXYZRGBA roi_pt;
            roi_pt.x = pt_x;
            roi_pt.y = pt_y;
            roi_pt.z = pt_z;
            roi_pt.r = pt_r;
            roi_pt.g = pt_g;
            roi_pt.b = pt_b;
            c_3d_viewer_lidar_filtered->cloud->points.push_back(roi_pt);
        }
    }

    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(c_3d_viewer_lidar_filtered->cloud,"cloud");
    ui->qvtkWidget_lidar_data_filtered->update();
}

// SLOT -------------------------------------------------------------------
void G_MAIN_WINDOW::SLOT_C_T_SCENEUPDATE_2_MAIN()
{
    mtx_sceneupdate.lock();
    excalib_disp_img = ResizeByConst(excalib_ori_img,excalib_disp_img_size);

    if(excalib_disp_img_scene->pt_list.size() == 0)
    {
        ui->lineEdit_img_x_pt_1->setText("");
        ui->lineEdit_img_y_pt_1->setText("");
        ui->lineEdit_img_x_pt_2->setText("");
        ui->lineEdit_img_y_pt_2->setText("");
        ui->lineEdit_img_x_pt_3->setText("");
        ui->lineEdit_img_y_pt_3->setText("");
    }
    else
    {
        for(uint pt_ind = 0; pt_ind < excalib_disp_img_scene->pt_list.size();pt_ind++)
        {
            cv::Point circle_center;
            circle_center.x = excalib_disp_img_scene->pt_list.at(pt_ind).x;
            circle_center.y = excalib_disp_img_scene->pt_list.at(pt_ind).y;
            cv::circle(excalib_disp_img,circle_center,4,cv::Scalar(0,0,255),2);

            switch(pt_ind)
            {
            case 0:
                ui->lineEdit_img_x_pt_1->setText(QString::number((double)circle_center.x/(double)m_excalib_resize_ratio_total));
                ui->lineEdit_img_y_pt_1->setText(QString::number((double)circle_center.y/(double)m_excalib_resize_ratio_total));
                break;
            case 1:
                ui->lineEdit_img_x_pt_2->setText(QString::number((double)circle_center.x/(double)m_excalib_resize_ratio_total));
                ui->lineEdit_img_y_pt_2->setText(QString::number((double)circle_center.y/(double)m_excalib_resize_ratio_total));
                break;
            case 2:
                ui->lineEdit_img_x_pt_3->setText(QString::number((double)circle_center.x/(double)m_excalib_resize_ratio_total));
                ui->lineEdit_img_y_pt_3->setText(QString::number((double)circle_center.y/(double)m_excalib_resize_ratio_total));
                break;
            }


            if(pt_ind >= 1)
            {
                cv::line(excalib_disp_img,cv::Point(excalib_disp_img_scene->pt_list.at(pt_ind-1).x,excalib_disp_img_scene->pt_list.at(pt_ind-1).y),cv::Point(excalib_disp_img_scene->pt_list.at(pt_ind).x,excalib_disp_img_scene->pt_list.at(pt_ind).y),cv::Scalar(0,0,255),2);
            }
            if(pt_ind == 2)
            {
                cv::line(excalib_disp_img,cv::Point(excalib_disp_img_scene->pt_list.at(pt_ind).x,excalib_disp_img_scene->pt_list.at(pt_ind).y),cv::Point(excalib_disp_img_scene->pt_list.at(0).x,excalib_disp_img_scene->pt_list.at(0).y),cv::Scalar(0,0,255),2);
            }
        }
    }
    excalib_disp_img_q = Mat2QImage(excalib_disp_img);
    excalib_disp_img_p.convertFromImage(excalib_disp_img_q);
    excalib_disp_img_scene->clear();
    excalib_disp_img_scene->addPixmap(excalib_disp_img_p);

    ui->graphicsView_excalib_img_load->setScene(excalib_disp_img_scene);
    ui->graphicsView_excalib_img_load->show();
    mtx_sceneupdate.unlock();
}

void G_MAIN_WINDOW::SLOT_C_T_SCENEUPDATE_2_MAIN_IPM()
{
    mtx_sceneupdate.lock();

    cv::Mat ipm_src_cp;
    ipm_src_img.copyTo(ipm_src_cp);

    if(ipm_src_img_scene->pt_list.size() == 0)
    {
        ui->lineEdit_ipm_pt_1_x->setText("");
        ui->lineEdit_ipm_pt_1_y->setText("");
        ui->lineEdit_ipm_pt_2_x->setText("");
        ui->lineEdit_ipm_pt_2_y->setText("");
        ui->lineEdit_ipm_pt_3_x->setText("");
        ui->lineEdit_ipm_pt_3_y->setText("");
        ui->lineEdit_ipm_pt_4_x->setText("");
        ui->lineEdit_ipm_pt_4_y->setText("");
    }

    for(uint pt_ind =0; pt_ind < ipm_src_img_scene->pt_list.size(); pt_ind++)
    {
        double img_x = (double)ipm_src_img_scene->pt_list[pt_ind].x/ipm_disp_about_src_ratio;
        double img_y = (double)ipm_src_img_scene->pt_list[pt_ind].y/ipm_disp_about_src_ratio;

        cv::Point circle_center;
        circle_center.x = (int)img_x;
        circle_center.y = (int)img_y;
        cv::circle(ipm_src_cp,circle_center,4,cv::Scalar(0,0,255),2);


        if(pt_ind >= 1)
        {
            cv::line(ipm_src_cp,cv::Point(ipm_src_img_scene->pt_list.at(pt_ind-1).x/ipm_disp_about_src_ratio,ipm_src_img_scene->pt_list.at(pt_ind-1).y/ipm_disp_about_src_ratio),cv::Point(ipm_src_img_scene->pt_list.at(pt_ind).x/ipm_disp_about_src_ratio,ipm_src_img_scene->pt_list.at(pt_ind).y/ipm_disp_about_src_ratio),cv::Scalar(0,0,255),2);
        }
        if(pt_ind == 3)
        {
            cv::line(ipm_src_cp,cv::Point(ipm_src_img_scene->pt_list.at(pt_ind).x/ipm_disp_about_src_ratio,ipm_src_img_scene->pt_list.at(pt_ind).y/ipm_disp_about_src_ratio),cv::Point(ipm_src_img_scene->pt_list.at(0).x/ipm_disp_about_src_ratio,ipm_src_img_scene->pt_list.at(0).y/ipm_disp_about_src_ratio),cv::Scalar(0,0,255),2);
        }

        switch(pt_ind)
        {
        case 0:
            ui->lineEdit_ipm_pt_1_x->setText(QString::number(img_x));
            ui->lineEdit_ipm_pt_1_y->setText(QString::number(img_y));
            break;
        case 1:
            ui->lineEdit_ipm_pt_2_x->setText(QString::number(img_x));
            ui->lineEdit_ipm_pt_2_y->setText(QString::number(img_y));
            break;
        case 2:
            ui->lineEdit_ipm_pt_3_x->setText(QString::number(img_x));
            ui->lineEdit_ipm_pt_3_y->setText(QString::number(img_y));
            break;
        case 3:
            ui->lineEdit_ipm_pt_4_x->setText(QString::number(img_x));
            ui->lineEdit_ipm_pt_4_y->setText(QString::number(img_y));
            break;
        }

    }

    cv::Size resize_const;
    resize_const.width = ui->graphicsView_ipm_src->geometry().width();
    resize_const.height = ui->graphicsView_ipm_src->geometry().height();

    ipm_src_img_disp = ResizeByConst(ipm_src_cp,resize_const);

    ipm_src_img_q = Mat2QImage(ipm_src_img_disp);
    ipm_src_img_p.convertFromImage(ipm_src_img_q);
    ipm_src_img_scene->clear();
    ipm_src_img_scene->addPixmap(ipm_src_img_p);

    ui->graphicsView_ipm_src->setScene(ipm_src_img_scene);
    ui->graphicsView_ipm_src->show();

    mtx_sceneupdate.unlock();
}

// ------------------------------------------------------------------------

void G_MAIN_WINDOW::on_pushButton_fitting_plane_model_clicked()
{
    ResetFilteredCloud();
    c_3d_viewer_lidar_filtered->viewer->removeAllShapes();
    m_plane_coefficients.reset(new pcl::ModelCoefficients);
    m_plane_inliers.reset(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);

    seg.setInputCloud (c_3d_viewer_lidar_filtered->cloud);
    seg.segment (*m_plane_inliers, *m_plane_coefficients);

    c_3d_viewer_lidar_filtered->viewer->addPlane(*m_plane_coefficients,"plane");
}

void G_MAIN_WINDOW::on_pushButton_projection_inlier_2_plane_clicked()
{
    c_3d_viewer_lidar_filtered->viewer->removeAllShapes();

    double A = m_plane_coefficients->values[0];
    double B = m_plane_coefficients->values[1];
    double C = m_plane_coefficients->values[2];
    double D = m_plane_coefficients->values[3];

    m_cloud_projection.reset(new PointCloudT);
    for(size_t i=0; i < m_plane_inliers->indices.size();++i)
    {
        double pt_x = c_3d_viewer_lidar_filtered->cloud->points[m_plane_inliers->indices[i]].x;
        double pt_y = c_3d_viewer_lidar_filtered->cloud->points[m_plane_inliers->indices[i]].y;
        double pt_z = c_3d_viewer_lidar_filtered->cloud->points[m_plane_inliers->indices[i]].z;

        double t = (-D - A*pt_x - B*pt_y - C*pt_z)/(A*A + B*B + C*C);

        double pt_proj_x = pt_x + t*A;
        double pt_proj_y = pt_y + t*B;
        double pt_proj_z = pt_z + t*C;

        pcl::PointXYZRGBA pt_proj;
        pt_proj.x = pt_proj_x;
        pt_proj.y = pt_proj_y;
        pt_proj.z = pt_proj_z;
        pt_proj.r = 0;
        pt_proj.g = 0;
        pt_proj.b = 255;
        m_cloud_projection->points.push_back(pt_proj);
    }

    c_3d_viewer_lidar_filtered->viewer->removePointCloud("cloud");
//    c_3d_viewer_lidar_filtered->cloud.reset(new PointCloudT);
//    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(c_3d_viewer_lidar_filtered->cloud,"cloud");
    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_projection,"cloud_proj");
    ui->qvtkWidget_lidar_data_filtered->update();

    m_point_ind = 0;
    m_point_max = m_cloud_projection->points.size() - 1;

    ui->horizontalSlider_lidar_data->setMaximum(m_point_max);
    ui->horizontalSlider_lidar_data->setValue(m_point_ind);

    disp_current_point(m_cloud_projection,c_3d_viewer_lidar_filtered->viewer,ui->qvtkWidget_lidar_data_filtered);
}

void G_MAIN_WINDOW::disp_current_point(PointCloudT::Ptr _cloud,boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer, QVTKWidget* _qvtkwidget)
{
    ui->lineEdit_point_ind->setText(QString::number(m_point_ind));
    ui->lineEdit_point_total->setText(QString::number(m_point_max));

    ui->lineEdit_point_x->setText(QString::number(_cloud->points[m_point_ind].x));
    ui->lineEdit_point_y->setText(QString::number(_cloud->points[m_point_ind].y));
    ui->lineEdit_point_z->setText(QString::number(_cloud->points[m_point_ind].z));

    for(size_t i=0;i < _cloud->points.size();i++)
    {
        if ( i == m_point_ind)
        {
            _cloud->points[i].r = 255;
            _cloud->points[i].g = 0;
            _cloud->points[i].b = 0;
        }
        else
        {
            _cloud->points[i].r = 0;
            _cloud->points[i].g = 0;
            _cloud->points[i].b = 255;
        }
    }
    _viewer->updatePointCloud(_cloud,"cloud_proj");
    _qvtkwidget->update();
}

void G_MAIN_WINDOW::on_horizontalSlider_lidar_data_sliderMoved(int position)
{
    m_point_ind = ui->horizontalSlider_lidar_data->value();

    disp_current_point(m_cloud_projection,c_3d_viewer_lidar_filtered->viewer,ui->qvtkWidget_lidar_data_filtered);
}

void G_MAIN_WINDOW::on_pushButton_excalib_next_clicked()
{
    if(m_point_ind + 1 > m_point_max)
    {
        m_point_ind = m_point_max;
    }
    else
    {
        m_point_ind++;
    }

    ui->horizontalSlider_lidar_data->setValue(m_point_ind);
    disp_current_point(m_cloud_projection,c_3d_viewer_lidar_filtered->viewer,ui->qvtkWidget_lidar_data_filtered);
}

void G_MAIN_WINDOW::on_pushButton_excalib_prev_clicked()
{
    if(m_point_ind -1 < 0)
    {
        m_point_ind = 0;
    }
    else
    {
        m_point_ind--;
    }
    ui->horizontalSlider_lidar_data->setValue(m_point_ind);
    disp_current_point(m_cloud_projection,c_3d_viewer_lidar_filtered->viewer,ui->qvtkWidget_lidar_data_filtered);
}

void G_MAIN_WINDOW::on_pushButton_extrapolation_clicked()
{
    ulong extra_ind_from = ui->lineEdit_extra_ind_from->text().toLong();
    ulong extra_ind_to = ui->lineEdit_extra_ind_to->text().toLong();

    double extra_x_to = m_cloud_projection->points[extra_ind_to].x;
    double extra_y_to = m_cloud_projection->points[extra_ind_to].y;
    double extra_z_to = m_cloud_projection->points[extra_ind_to].z;

    double extra_x_from = m_cloud_projection->points[extra_ind_from].x;
    double extra_y_from = m_cloud_projection->points[extra_ind_from].y;
    double extra_z_from = m_cloud_projection->points[extra_ind_from].z;

    double x_direct_vec = extra_x_to - extra_x_from;
    double y_direct_vec = extra_y_to - extra_y_from;
    double z_direct_vec = extra_z_to - extra_z_from;

    double extra_x = extra_x_to + 0.5*x_direct_vec;
    double extra_y = extra_y_to + 0.5*y_direct_vec;
    double extra_z = extra_z_to + 0.5*z_direct_vec;

    pcl::PointXYZRGBA pt_extra;

    if(ui->radioButton_extra_left->isChecked())
    {
        pt_extra.x = extra_x;
        pt_extra.y = extra_y;
        pt_extra.z = extra_z;
        pt_extra.r = 255;
        pt_extra.g = 255;
        pt_extra.b = 0;
        m_cloud_left_side->points.push_back(pt_extra);
        c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_left_side,"cloud_left_side");
    }
    else
    {
        pt_extra.x = extra_x;
        pt_extra.y = extra_y;
        pt_extra.z = extra_z;
        pt_extra.r = 0;
        pt_extra.g = 255;
        pt_extra.b = 0;
        m_cloud_right_side->points.push_back(pt_extra);
        c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_right_side,"cloud_right_side");
    }
    ui->qvtkWidget_lidar_data_filtered->update();
}

void G_MAIN_WINDOW::on_pushButton_fitting_triangle_clicked()
{
    m_left_line_coefficients.reset(new pcl::ModelCoefficients);
    m_left_line_inliers.reset(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg_left;
    // Optional
    seg_left.setOptimizeCoefficients (true);
    // Mandatory
    seg_left.setModelType (pcl::SACMODEL_LINE);
    seg_left.setMethodType (pcl::SAC_RANSAC);
    seg_left.setDistanceThreshold (0.02);

    seg_left.setInputCloud (m_cloud_left_side);
    seg_left.segment (*m_left_line_inliers, *m_left_line_coefficients);

    m_right_line_coefficients.reset(new pcl::ModelCoefficients);
    m_right_line_inliers.reset(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg_right;
    // Optional
    seg_right.setOptimizeCoefficients (true);
    // Mandatory
    seg_right.setModelType (pcl::SACMODEL_LINE);
    seg_right.setMethodType (pcl::SAC_RANSAC);
    seg_right.setDistanceThreshold (0.02);

    seg_right.setInputCloud (m_cloud_right_side);
    seg_right.segment (*m_right_line_inliers, *m_right_line_coefficients);

    const std::string id_l = "left_line";
    const std::string id_r = "right_line";
    c_3d_viewer_lidar_filtered->viewer->addLine(*m_left_line_coefficients,id_l);
    c_3d_viewer_lidar_filtered->viewer->addLine(*m_right_line_coefficients,id_r);

    cv::Point3f vec_lp;
    vec_lp.x = m_left_line_coefficients->values[0];
    vec_lp.y = m_left_line_coefficients->values[1];
    vec_lp.z = m_left_line_coefficients->values[2];

    cv::Point3f vec_rp;
    vec_rp.x = m_right_line_coefficients->values[0];
    vec_rp.y = m_right_line_coefficients->values[1];
    vec_rp.z = m_right_line_coefficients->values[2];

    cv::Point3f vec_e;
    vec_e.x = m_left_line_coefficients->values[3];
    vec_e.y = m_left_line_coefficients->values[4];
    vec_e.z = m_left_line_coefficients->values[5];

    cv::Point3f vec_f;
    vec_f.x = m_right_line_coefficients->values[3];
    vec_f.y = m_right_line_coefficients->values[4];
    vec_f.z = m_right_line_coefficients->values[5];

    cv::Point3f vec_lr;
    vec_lr.x = vec_rp.x - vec_lp.x;
    vec_lr.y = vec_rp.y - vec_lp.y;
    vec_lr.z = vec_rp.z - vec_lp.z;

    cv::Point3f vec_f_cross_g;
    m_vec_cal_obj.CrossProd3D(vec_f,vec_lr,vec_f_cross_g);

    cv::Point3f vec_f_cross_e;
    m_vec_cal_obj.CrossProd3D(vec_f,vec_e,vec_f_cross_e);

    cv::Point3f vec_intersection;

    cv::Point3f vec_f_cross_lr;
    float norm_f_cross_lr = 0;
    float norm_f_cross_e = 0;
    float e_scale_factor;

    m_vec_cal_obj.CrossProd3D(vec_f,vec_lr,vec_f_cross_lr);
    norm_f_cross_lr = m_vec_cal_obj.Norm3D(vec_f_cross_lr);
    norm_f_cross_e = m_vec_cal_obj.Norm3D(vec_f_cross_e);

    e_scale_factor = norm_f_cross_lr/norm_f_cross_e;

    cv::Point3f vec_e_scale;
    vec_e_scale.x = e_scale_factor*vec_e.x;
    vec_e_scale.y = e_scale_factor*vec_e.y;
    vec_e_scale.z = e_scale_factor*vec_e.z;

    if(vec_f_cross_g.x * vec_f_cross_e.x > 0) // same direction
    {
        vec_intersection = m_vec_cal_obj.Sum3D(vec_lp,vec_e_scale);
    }
    else
    {
        vec_intersection = m_vec_cal_obj.Sum3D(vec_lp,-vec_e_scale);
    }

    cv::Point3f vec_intersection_to_lp;
    vec_intersection_to_lp.x = vec_lp.x - vec_intersection.x;
    vec_intersection_to_lp.y = vec_lp.y - vec_intersection.y;
    vec_intersection_to_lp.z = vec_lp.z - vec_intersection.z;

    if(vec_intersection_to_lp.x * vec_e.x > 0)
    {
        vec_e.x = -vec_e.x;
        vec_e.y = -vec_e.y;
        vec_e.z = -vec_e.z;
    }



    pcl::PointXYZRGBA pt_intersection_lr;
    pt_intersection_lr.x = vec_intersection.x;
    pt_intersection_lr.y = vec_intersection.y;
    pt_intersection_lr.z = vec_intersection.z;
    pt_intersection_lr.r = 200;
    pt_intersection_lr.g = 100;
    pt_intersection_lr.b = 50;

    m_cloud_triangle_vertice->points.push_back(pt_intersection_lr);

    cv::Point3f pt_left_bottom;
    cv::Point3f pt_right_bottom;

    float left_side_length = ui->lineEdit_caliboard_l->text().toFloat();
    float right_side_length = ui->lineEdit_caliboard_r->text().toFloat();
    float bottom_side_length = ui->lineEdit_caliboard_b->text().toFloat();

    pt_left_bottom.x = pt_intersection_lr.x + left_side_length*(-vec_e.x);
    pt_left_bottom.y = pt_intersection_lr.y + left_side_length*(-vec_e.y);
    pt_left_bottom.z = pt_intersection_lr.z + left_side_length*(-vec_e.z);

    float between_ang = m_vec_cal_obj.CalBetweenAngle(left_side_length,right_side_length,bottom_side_length);

    cv::Point3f inv_vec_e;
    inv_vec_e.x = -vec_e.x;
    inv_vec_e.y = -vec_e.y;
    inv_vec_e.z = -vec_e.z;

    cv::Point3f plane_norm;
    plane_norm.x = m_plane_coefficients->values[0];
    plane_norm.y = m_plane_coefficients->values[1];
    plane_norm.z = m_plane_coefficients->values[2];

    m_plane_norm.x = plane_norm.x;
    m_plane_norm.y = plane_norm.y;
    m_plane_norm.z = plane_norm.z;

    cv::Point3f top_to_right_dir_vec;

//    top_to_right_dir_vec = m_vec_cal_obj.ObtainOtherSideTri(inv_vec_e,between_ang,plane_norm);

    top_to_right_dir_vec = m_vec_cal_obj.RotateByArbitraryAxis(inv_vec_e,between_ang,plane_norm);

    pt_right_bottom.x = pt_intersection_lr.x + right_side_length*(top_to_right_dir_vec.x);
    pt_right_bottom.y = pt_intersection_lr.y + right_side_length*(top_to_right_dir_vec.y);
    pt_right_bottom.z = pt_intersection_lr.z + right_side_length*(top_to_right_dir_vec.z);

    // Find u and v vector

    cv::Point3f vec_i;
    vec_i.x = 1.0;
    vec_i.y = 0;
    vec_i.z = 0;

    cv::Point3f vec_i_minus_a;
    vec_i_minus_a.x = vec_i.x - plane_norm.x;
    vec_i_minus_a.y = vec_i.y - plane_norm.y;
    vec_i_minus_a.z = vec_i.z - plane_norm.z;

    float i_minus_a_inner_prod_a;
    i_minus_a_inner_prod_a = m_vec_cal_obj.InnerProd3D(vec_i_minus_a,plane_norm);

    float direction_scalar = -1/i_minus_a_inner_prod_a;

    cv::Point3f vec_u_wo_normalize;
    vec_u_wo_normalize.x = plane_norm.x + direction_scalar*(vec_i.x - plane_norm.x);
    vec_u_wo_normalize.y = plane_norm.y + direction_scalar*(vec_i.y - plane_norm.y);
    vec_u_wo_normalize.z = plane_norm.z + direction_scalar*(vec_i.z - plane_norm.z);

    float vec_u_wo_normalize_length = sqrt(vec_u_wo_normalize.x*vec_u_wo_normalize.x + vec_u_wo_normalize.y*vec_u_wo_normalize.y + vec_u_wo_normalize.z*vec_u_wo_normalize.z);
    m_u_vec.x = vec_u_wo_normalize.x/vec_u_wo_normalize_length;
    m_u_vec.y = vec_u_wo_normalize.y/vec_u_wo_normalize_length;
    m_u_vec.z = vec_u_wo_normalize.z/vec_u_wo_normalize_length;

    m_v_vec = m_vec_cal_obj.RotateByArbitraryAxis(m_u_vec,(3.1415926535)/2.0,plane_norm);

    pcl::PointXYZRGBA pt_left_bottom_pcl;
    pt_left_bottom_pcl.x = pt_left_bottom.x;
    pt_left_bottom_pcl.y = pt_left_bottom.y;
    pt_left_bottom_pcl.z = pt_left_bottom.z;
    pt_left_bottom_pcl.r = 200;
    pt_left_bottom_pcl.g = 100;
    pt_left_bottom_pcl.b = 50;

    pcl::PointXYZRGBA pt_right_bottom_pcl;
    pt_right_bottom_pcl.x = pt_right_bottom.x;
    pt_right_bottom_pcl.y = pt_right_bottom.y;
    pt_right_bottom_pcl.z = pt_right_bottom.z;
    pt_right_bottom_pcl.r = 200;
    pt_right_bottom_pcl.g = 100;
    pt_right_bottom_pcl.b = 50;

    m_cloud_triangle_vertice->points.push_back(pt_left_bottom_pcl);
    m_cloud_triangle_vertice->points.push_back(pt_right_bottom_pcl);

    cout << "top : (" << m_cloud_triangle_vertice->points[0].x << ", " << m_cloud_triangle_vertice->points[0].y << ", " << m_cloud_triangle_vertice->points[0].z << ")" << endl;
    cout << "bottom left : (" << m_cloud_triangle_vertice->points[1].x << ", " << m_cloud_triangle_vertice->points[1].y << ", " << m_cloud_triangle_vertice->points[1].z << ")" << endl;
    cout << "bottom right : (" << m_cloud_triangle_vertice->points[2].x << ", " << m_cloud_triangle_vertice->points[2].y << ", " << m_cloud_triangle_vertice->points[2].z << ")" << endl;

    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_triangle_vertice,"cloud_triangle_vertice");
    c_3d_viewer_lidar_filtered->viewer->addPolygon<pcl::PointXYZRGBA>(m_cloud_triangle_vertice,1.0,0.0,0.0,"triangle",0);
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"triangle");
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "triangle");
    ui->qvtkWidget_lidar_data_filtered->update();
}

void G_MAIN_WINDOW::ResetFilteredCloud()
{
    m_cloud_projection.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_projection,"cloud_proj");

    m_cloud_left_side.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_left_side,"cloud_left_side");

    m_cloud_right_side.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_right_side,"cloud_right_side");

    m_cloud_triangle_vertice.reset(new PointCloudT);
    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_triangle_vertice,"cloud_triangle_vertice");

    ui->qvtkWidget_lidar_data_filtered->update();
}

void G_MAIN_WINDOW::on_pushButton_save_calibration_sample_clicked()
{
    QString directory;
    directory = QFileDialog::getSaveFileName(this,"Set Save file name") ;
    string save_calibration_sample_path = directory.toStdString();

    cv::Point3f lidar_pt_1;
    lidar_pt_1.x = m_cloud_triangle_vertice->points[0].x;
    lidar_pt_1.y = m_cloud_triangle_vertice->points[0].y;
    lidar_pt_1.z = m_cloud_triangle_vertice->points[0].z;

    cv::Point3f lidar_pt_2;
    lidar_pt_2.x = m_cloud_triangle_vertice->points[1].x;
    lidar_pt_2.y = m_cloud_triangle_vertice->points[1].y;
    lidar_pt_2.z = m_cloud_triangle_vertice->points[1].z;

    cv::Point3f lidar_pt_3;
    lidar_pt_3.x = m_cloud_triangle_vertice->points[2].x;
    lidar_pt_3.y = m_cloud_triangle_vertice->points[2].y;
    lidar_pt_3.z = m_cloud_triangle_vertice->points[2].z;

    cv::Point2f img_pt_1;
    img_pt_1.x = ui->lineEdit_img_x_pt_1->text().toFloat();
    img_pt_1.y = ui->lineEdit_img_y_pt_1->text().toFloat();

    cv::Point2f img_pt_2;
    img_pt_2.x = ui->lineEdit_img_x_pt_2->text().toFloat();
    img_pt_2.y = ui->lineEdit_img_y_pt_2->text().toFloat();

    cv::Point2f img_pt_3;
    img_pt_3.x = ui->lineEdit_img_x_pt_3->text().toFloat();
    img_pt_3.y = ui->lineEdit_img_y_pt_3->text().toFloat();

    FileStorage fs(save_calibration_sample_path,FileStorage::WRITE);
    fs << "lidar_pt_1" << lidar_pt_1;
    fs << "lidar_pt_2" << lidar_pt_2;
    fs << "lidar_pt_3" << lidar_pt_3;
    fs << "img_pt_1" << img_pt_1;
    fs << "img_pt_2" << img_pt_2;
    fs << "img_pt_3" << img_pt_3;
    fs.release();
}

void G_MAIN_WINDOW::on_checkBox_show_filtered_pts_clicked()
{
    if(ui->checkBox_show_filtered_pts->isChecked())
    {
        c_3d_viewer_lidar_filtered->viewer->addPointCloud(c_3d_viewer_lidar_filtered->cloud,"cloud");
        c_3d_viewer_lidar_filtered->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud");
    }
    else
    {
        c_3d_viewer_lidar_filtered->viewer->removePointCloud("cloud");
    }
    ui->qvtkWidget_lidar_data_filtered->update();
}

void G_MAIN_WINDOW::MoveTriangle()
{
    float _u = ui->lineEdit_triangle_move_u->text().toFloat();
    float _v = ui->lineEdit_triangle_move_v->text().toFloat();

    for(size_t i=0;i < m_cloud_triangle_vertice->points.size();i++)
    {
        m_cloud_triangle_vertice->points[i].x = m_cloud_triangle_vertice->points[i].x + _u*m_u_vec.x + _v*m_v_vec.x;
        m_cloud_triangle_vertice->points[i].y = m_cloud_triangle_vertice->points[i].y + _u*m_u_vec.y + _v*m_v_vec.y;
        m_cloud_triangle_vertice->points[i].z = m_cloud_triangle_vertice->points[i].z + _u*m_u_vec.z + _v*m_v_vec.z;
    }

    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_triangle_vertice,"cloud_triangle_vertice");
    c_3d_viewer_lidar_filtered->viewer->removeShape("triangle");
    c_3d_viewer_lidar_filtered->viewer->addPolygon<pcl::PointXYZRGBA>(m_cloud_triangle_vertice,1.0,0.0,0.0,"triangle",0);
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"triangle");
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "triangle");
    ui->qvtkWidget_lidar_data_filtered->update();
}


void G_MAIN_WINDOW::RotateTraingle()
{
    cv::Point3f redistrb_bottom_right_pt;
    cv::Point3f redistrb_top_pt;

    redistrb_bottom_right_pt.x = m_cloud_triangle_vertice->points[2].x - m_cloud_triangle_vertice->points[1].x;
    redistrb_bottom_right_pt.y = m_cloud_triangle_vertice->points[2].y - m_cloud_triangle_vertice->points[1].y;
    redistrb_bottom_right_pt.z = m_cloud_triangle_vertice->points[2].z - m_cloud_triangle_vertice->points[1].z;

    redistrb_top_pt.x =m_cloud_triangle_vertice->points[0].x - m_cloud_triangle_vertice->points[1].x;
    redistrb_top_pt.y =m_cloud_triangle_vertice->points[0].y - m_cloud_triangle_vertice->points[1].y;
    redistrb_top_pt.z =m_cloud_triangle_vertice->points[0].z - m_cloud_triangle_vertice->points[1].z;

    cv::Point3f rotate_redistrb_bottom_right_pt;
    cv::Point3f rotate_redistrb_top_pt;

    float angle = ui->lineEdit_triangle_rotate_angle->text().toFloat();

    rotate_redistrb_bottom_right_pt = m_vec_cal_obj.RotateByArbitraryAxis(redistrb_bottom_right_pt,angle,m_plane_norm);
    rotate_redistrb_top_pt = m_vec_cal_obj.RotateByArbitraryAxis(redistrb_top_pt,angle,m_plane_norm);

    cv::Point3f new_bottom_right_pt;
    cv::Point3f new_top_pt;

    new_bottom_right_pt.x = rotate_redistrb_bottom_right_pt.x + m_cloud_triangle_vertice->points[1].x;
    new_bottom_right_pt.y = rotate_redistrb_bottom_right_pt.y + m_cloud_triangle_vertice->points[1].y;
    new_bottom_right_pt.z = rotate_redistrb_bottom_right_pt.z + m_cloud_triangle_vertice->points[1].z;

    new_top_pt.x = rotate_redistrb_top_pt.x + m_cloud_triangle_vertice->points[1].x;
    new_top_pt.y = rotate_redistrb_top_pt.y + m_cloud_triangle_vertice->points[1].y;
    new_top_pt.z = rotate_redistrb_top_pt.z + m_cloud_triangle_vertice->points[1].z;

    m_cloud_triangle_vertice->points[2].x = new_bottom_right_pt.x;
    m_cloud_triangle_vertice->points[2].y = new_bottom_right_pt.y;
    m_cloud_triangle_vertice->points[2].z = new_bottom_right_pt.z;

    m_cloud_triangle_vertice->points[0].x = new_top_pt.x;
    m_cloud_triangle_vertice->points[0].y = new_top_pt.y;
    m_cloud_triangle_vertice->points[0].z = new_top_pt.z;

    c_3d_viewer_lidar_filtered->viewer->updatePointCloud(m_cloud_triangle_vertice,"cloud_triangle_vertice");
    c_3d_viewer_lidar_filtered->viewer->removeShape("triangle");
    c_3d_viewer_lidar_filtered->viewer->addPolygon<pcl::PointXYZRGBA>(m_cloud_triangle_vertice,1.0,0.0,0.0,"triangle",0);
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"triangle");
    c_3d_viewer_lidar_filtered->viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "triangle");
    ui->qvtkWidget_lidar_data_filtered->update();
}

void G_MAIN_WINDOW::on_pushButton_move_triangle_clicked()
{
    MoveTriangle();
}

void G_MAIN_WINDOW::on_pushButton_rotate_triangle_clicked()
{
    RotateTraingle();
}

void G_MAIN_WINDOW::on_pushButton_load_calibration_samples_clicked()
{
    QStringList directory_list;
    directory_list = QFileDialog::getOpenFileNames(this,"Select calibration samples");

    vector<cv::Point3f> lidar_point_list;
    vector<cv::Point2f> img_point_list;

    for(size_t i = 0; i < directory_list.size();i++)
    {
        QString file_path;
        file_path = directory_list[i];

        string file_path_str = file_path.toStdString();

        cv::Point3f lidar_pt_1;
        cv::Point3f lidar_pt_2;
        cv::Point3f lidar_pt_3;
        cv::Point2f img_pt_1;
        cv::Point2f img_pt_2;
        cv::Point2f img_pt_3;
        cv::FileStorage fs(file_path_str,cv::FileStorage::READ);
        fs["lidar_pt_1"] >> lidar_pt_1;
        fs["lidar_pt_2"] >> lidar_pt_2;
        fs["lidar_pt_3"] >> lidar_pt_3;
        fs["img_pt_1"] >> img_pt_1;
        fs["img_pt_2"] >> img_pt_2;
        fs["img_pt_3"] >> img_pt_3;
        fs.release();

        lidar_point_list.push_back(lidar_pt_1);
        lidar_point_list.push_back(lidar_pt_2);
        lidar_point_list.push_back(lidar_pt_3);
        img_point_list.push_back(img_pt_1);
        img_point_list.push_back(img_pt_2);
        img_point_list.push_back(img_pt_3);
    }

//    arma::mat A(2lidar_point_list.size(),12);
//    arma::mat B(2*lidar_point_list.size(),1);

    arma::mat A(lidar_point_list.size(),12);
    arma::mat B(lidar_point_list.size(),1);

    arma::mat M(12,1);
    B.zeros();
    for(size_t i = 0;i < lidar_point_list.size();i++)
    {
//        A(2*i,0) = lidar_point_list[i].x;
//        A(2*i,1) = lidar_point_list[i].y;
//        A(2*i,2) = lidar_point_list[i].z;
//        A(2*i,3) = 1;
//        A(2*i,4) = 0;
//        A(2*i,5) = 0;
//        A(2*i,6) = 0;
//        A(2*i,7) = 0;
//        A(2*i,8) = -img_point_list[i].x*lidar_point_list[i].x;
//        A(2*i,9) = -img_point_list[i].x*lidar_point_list[i].y;
//        A(2*i,10) = -img_point_list[i].x*lidar_point_list[i].z;
//        A(2*i,11) = -img_point_list[i].x;

//        A(2*i+1,0) = 0;
//        A(2*i+1,1) = 0;
//        A(2*i+1,2) = 0;
//        A(2*i+1,3) = 0;
//        A(2*i+1,4) = lidar_point_list[i].x;
//        A(2*i+1,5) = lidar_point_list[i].y;
//        A(2*i+1,6) = lidar_point_list[i].z;
//        A(2*i+1,7) = 1;
//        A(2*i+1,8) = -img_point_list[i].y*lidar_point_list[i].x;
//        A(2*i+1,9) = -img_point_list[i].y*lidar_point_list[i].y;
//        A(2*i+1,10) = -img_point_list[i].y*lidar_point_list[i].z;
//        A(2*i+1,11) = -img_point_list[i].y;

        A(i,0) = lidar_point_list[i].x;
        A(i,1) = lidar_point_list[i].y;
        A(i,2) = lidar_point_list[i].z;
        A(i,3) = 1;
        A(i,4) = 0;
        A(i,5) = 0;
        A(i,6) = 0;
        A(i,7) = 0;
        A(i,8) = -img_point_list[i].x*lidar_point_list[i].x;
        A(i,9) = -img_point_list[i].x*lidar_point_list[i].y;
        A(i,10) = -img_point_list[i].x*lidar_point_list[i].z;
        A(i,11) = -img_point_list[i].x;
    }

    arma::mat U;
    arma::vec s;
    arma::mat V;

    arma::svd(U,s,V,A);

    m_transform_mat = cv::Mat2f::zeros(3,4);

    for(size_t i = 0; i < 12 ;i++)
    {
        M(i,0) = V(i,V.n_cols-1);

        int num_of_row = i/4;
        int num_of_cols = i%4;

        m_transform_mat(num_of_row,num_of_cols) = M(i,0);

//        cout << M(i,0) << endl;
    }

    arma::mat M_check(3,4);

    for(int i = 0; i < 3;i++)
    {
        for(int j = 0; j < 4;j++)
        {
            M_check(i,j) = m_transform_mat(i,j);
        }
    }

    arma::mat check_mat(12,1);

    check_mat = A*M;
    for (int i =0;i<12;i++)
    {
        cout << check_mat(i,0) << endl;
    }

    for(size_t i = 0;i < lidar_point_list.size();i++)
    {
        arma::mat LIDAR(4,1);
        arma::mat IMG(3,1);

        LIDAR(0,0) = lidar_point_list[i].x;
        LIDAR(1,0) = lidar_point_list[i].y;
        LIDAR(2,0) = lidar_point_list[i].z;
        LIDAR(3,0) = 1;

        IMG = M_check*LIDAR;

        float mul_x = IMG(0,0);
        float mul_y = IMG(1,0);

        cout << "ratio x : " << (float)img_point_list[i].x /mul_x << " , ratio y : " << (float)img_point_list[i].y /mul_y << endl;
    }

    cout << "calculation finished!" << endl;
}

void G_MAIN_WINDOW::on_pushButton_save_calibration_result_clicked()
{
    QString directory;
    directory = QFileDialog::getSaveFileName(this,"Set Save file name") ;
    string save_calibration_result_path = directory.toStdString();

    FileStorage fs(save_calibration_result_path,FileStorage::WRITE);
    fs << "transform_mat" << m_transform_mat;
    fs.release();
}

void G_MAIN_WINDOW::on_pushButton_load_calibration_result_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select Calibration result");

    string calibration_result_path = directory.toStdString();

    cv::FileStorage fs(calibration_result_path,cv::FileStorage::READ);
    fs["transform_mat"] >> m_transform_mat;

}

void G_MAIN_WINDOW::on_pushButton_display_result_clicked()
{
    uint img_cols = ui->lineEdit_camera_frame_width->text().toInt();
    uint img_rows = ui->lineEdit_camera_frame_height->text().toInt();

    cv::Size size_load_to_real;
    size_load_to_real.width = img_cols;
    size_load_to_real.height = img_rows;

    cv::Mat real_img;
    cv::resize(excalib_ori_img,real_img,size_load_to_real);

    arma::mat M(3,4);

    for(int i = 0; i < 3;i++)
    {
        for(int j = 0; j < 4;j++)
        {
            M(i,j) = m_transform_mat(i,j);
        }
    }

    for(size_t i=0;i < c_3d_viewer_lidar_load->cloud->points.size();i++)
    {
        arma::mat LIDAR(4,1);

        LIDAR(0,0) = c_3d_viewer_lidar_load->cloud->points[i].x;
        LIDAR(1,0) = c_3d_viewer_lidar_load->cloud->points[i].y;
        LIDAR(2,0) = c_3d_viewer_lidar_load->cloud->points[i].z;
        LIDAR(3,0) = 1;

        arma::mat IMG(3,1);

        IMG = M*LIDAR;

        if((IMG(0,0) >= 0) && (IMG(0,0) < real_img.cols) && (IMG(1,0) >= 0) && (IMG(1,0) < real_img.rows))
        {
//            cout << "x : " << IMG(0,0) << ", y : " << IMG(1,0) << endl;

            cv::circle(real_img,cv::Point(IMG(0,0),IMG(1,0)),2,cv::Scalar(0,0,255),2);
        }
    }

    cv::resize(real_img,disp_calib_result_img,cv::Size(960,604));

    disp_calib_result_img_q = Mat2QImage(disp_calib_result_img);
    disp_calib_result_img_p.convertFromImage(disp_calib_result_img_q);
    disp_calib_result_img_scene->clear();
    disp_calib_result_img_scene->addPixmap(disp_calib_result_img_p);

    ui->graphicsView_disp_calib_result->setScene(disp_calib_result_img_scene);
    ui->graphicsView_disp_calib_result->show();
}

void G_MAIN_WINDOW::on_pushButton_remap_clicked()
{
    cv::Size distort_size;

    distort_size.width = (double)ori_img.cols;
    distort_size.height = (double)ori_img.rows;

    m_calib_obj.initUndistortSet(distort_size);
    remap_img = m_calib_obj.ReMap(ori_img);

    cv::Mat remap_img_resize;

    cv::Size resize_size;

    resize_size.width = ui->graphicsView_img_remap->geometry().width();
    resize_size.height = ui->graphicsView_img_remap->geometry().height();

    remap_img_resize = ResizeByConst(remap_img,resize_size);

    remap_img_resize.copyTo(remap_disp_img);
    remap_disp_img_q = Mat2QImage(remap_disp_img);
    remap_disp_img_p.convertFromImage(remap_disp_img_q);
    remap_disp_img_scene->clear();
    remap_disp_img_scene->addPixmap(remap_disp_img_p);

    ui->graphicsView_img_remap->setScene(remap_disp_img_scene);
    ui->graphicsView_img_remap->show();

}

void G_MAIN_WINDOW::on_pushButton_load_ipm_src_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select ipm src");

    string ipm_src_path = directory.toStdString();

    cv::Mat ipm_src_load_img = cv::imread(ipm_src_path);

    if(ipm_src_load_img.empty())
        return;
    else
    {
        ipm_src_load_img.copyTo(ipm_src_img);
    }

    cv::Size resize_const;
    resize_const.width = ui->graphicsView_ipm_src->geometry().width();
    resize_const.height = ui->graphicsView_ipm_src->geometry().height();

    ipm_src_img_disp = ResizeByConst(ipm_src_img,resize_const, &ipm_disp_about_src_ratio);

    ipm_src_img_q = Mat2QImage(ipm_src_img_disp);
    ipm_src_img_p.convertFromImage(ipm_src_img_q);
    ipm_src_img_scene->clear();
    ipm_src_img_scene->addPixmap(ipm_src_img_p);

    ui->graphicsView_ipm_src->setScene(ipm_src_img_scene);
    ui->graphicsView_ipm_src->show();
}

void G_MAIN_WINDOW::on_pushButton_load_single_img_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select single img src");

    string single_img_src_path = directory.toStdString();

    ori_img = cv::imread(single_img_src_path);

    cv::Size ori_img_disp_const;

    ori_img_disp_const.width = ui->graphicsView_img_load->geometry().width();
    ori_img_disp_const.height = ui->graphicsView_img_load->geometry().height();

    disp_img = ResizeByConst(ori_img,ori_img_disp_const);
    disp_img_q = Mat2QImage(disp_img);
    disp_img_p.convertFromImage(disp_img_q);
    disp_img_scene->clear();
    disp_img_scene->addPixmap(disp_img_p);

    ui->graphicsView_img_load->setScene(disp_img_scene);
    ui->graphicsView_img_load->show();
}



void G_MAIN_WINDOW::on_pushButton_load_cam_model_file_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select camera model");

    string camera_model_path = directory.toStdString();

    cv::FileStorage fs(camera_model_path, cv::FileStorage::READ);
    fs["m_camera_matrix"] >> m_calib_obj.m_camera_matrix;
    fs["m_dist_coeffs"] >> m_calib_obj.m_dist_coeffs;
    fs.release();
}

void G_MAIN_WINDOW::on_pushButton_save_remap_img_clicked()
{
    QString directory;
    directory = QFileDialog::getSaveFileName(this,"Set Save file name") ;

    string save_remap_img_path = directory.toStdString();

    cv::imwrite(save_remap_img_path,remap_img);
}

void G_MAIN_WINDOW::on_pushButton_cal_ipm_clicked()
{
    cv::Mat ipm_lambda(2,4,CV_32FC1);
    cv::Point2f inputQuad_pt[4];
    cv::Point2f outputQuad_pt[4];

    inputQuad_pt[0] = cv::Point2f(ui->lineEdit_ipm_pt_1_x->text().toDouble(),ui->lineEdit_ipm_pt_1_y->text().toDouble());
    inputQuad_pt[1] = cv::Point2f(ui->lineEdit_ipm_pt_2_x->text().toDouble(),ui->lineEdit_ipm_pt_2_y->text().toDouble());
    inputQuad_pt[2] = cv::Point2f(ui->lineEdit_ipm_pt_3_x->text().toDouble(),ui->lineEdit_ipm_pt_3_y->text().toDouble());
    inputQuad_pt[3] = cv::Point2f(ui->lineEdit_ipm_pt_4_x->text().toDouble(),ui->lineEdit_ipm_pt_4_y->text().toDouble());

    ipm_lambda = cv::Mat::zeros(ipm_src_img.rows,ipm_src_img.cols,ipm_src_img.type());
    outputQuad_pt[0] = cv::Point2f(ui->lineEdit_ipm_pt_1_x_r->text().toDouble(),ui->lineEdit_ipm_pt_1_y_r->text().toDouble());
    outputQuad_pt[1] = cv::Point2f(ui->lineEdit_ipm_pt_2_x_r->text().toDouble(),ui->lineEdit_ipm_pt_2_y_r->text().toDouble());
    outputQuad_pt[2] = cv::Point2f(ui->lineEdit_ipm_pt_3_x_r->text().toDouble(),ui->lineEdit_ipm_pt_3_y_r->text().toDouble());
    outputQuad_pt[3] = cv::Point2f(ui->lineEdit_ipm_pt_4_x_r->text().toDouble(),ui->lineEdit_ipm_pt_4_y_r->text().toDouble());
    ipm_lambda = cv::getPerspectiveTransform(inputQuad_pt,outputQuad_pt);

    m_ipm_lambda = ipm_lambda;

    cv::Size ipm_disp_size;
    ipm_disp_size.width = 1000;
    ipm_disp_size.height = 1000;

    cv::warpPerspective(ipm_src_img,ipm_target_img,ipm_lambda,ipm_disp_size);

    ipm_target_img_q = Mat2QImage(ipm_target_img);
    ipm_target_img_p.convertFromImage(ipm_target_img_q);
    ipm_target_img_scene->clear();
    ipm_target_img_scene->addPixmap(ipm_target_img_p);

    ui->graphicsView_ipm_target->setScene(ipm_target_img_scene);
    ui->graphicsView_ipm_target->show();
}

void G_MAIN_WINDOW::on_pushButton_save_ipm_param_clicked()
{
    QString directory;
    directory = QFileDialog::getSaveFileName(this,"Set Save file name") ;
    string save_ipm_param_path = directory.toStdString();

    FileStorage fs(save_ipm_param_path,FileStorage::WRITE);
    fs << "m_ipm_lambda" << m_ipm_lambda;
    fs.release();
}

void G_MAIN_WINDOW::on_pushButton_save_ipm_img_clicked()
{
    QString directory;
    directory = QFileDialog::getSaveFileName(this,"Set Save file name") ;

    string save_ipm_img_path = directory.toStdString();

    cv::imwrite(save_ipm_img_path,ipm_target_img);
}

void G_MAIN_WINDOW::on_pushButton_cal_corr_clicked()
{
    int ori_x = ui->lineEdit_ori_x->text().toInt();
    int ori_y = ui->lineEdit_ori_y->text().toInt();

//    int ori_cen_x = ori_x - 960/2;
//    int ori_cen_y = ori_y - 604/2;

//    double r =  sqrt(ori_cen_x*ori_cen_x + ori_cen_y*ori_cen_y)/(double)sqrt((960/2)*(960/2) + (604/2)*(604/2));

//    cout <<  "r : " << r << endl;
//    double k1 = m_calib_obj.m_dist_coeffs.at<double>(0,0);
//    double k2 = m_calib_obj.m_dist_coeffs.at<double>(0,1);
//    double p1 = m_calib_obj.m_dist_coeffs.at<double>(0,2);
//    double p2 = m_calib_obj.m_dist_coeffs.at<double>(0,3);
//    double k3 = m_calib_obj.m_dist_coeffs.at<double>(0,4);
//    double k4 = m_calib_obj.m_dist_coeffs.at<double>(0,5);
//    double k5 = m_calib_obj.m_dist_coeffs.at<double>(0,6);
//    double k6 = m_calib_obj.m_dist_coeffs.at<double>(0,7);

////    cout << "k1 : " << k1 << endl;
////    cout << "k2 : " << k2 << endl;
////    cout << "k3 : " << k3 << endl;
////    cout << "k4 : " << k4 << endl;
////    cout << "k5 : " << k5 << endl;
////    cout << "k6 : " << k6 << endl;

//    double x_corr_r = (double)ori_cen_x*(1.0 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6) + k4*pow(r,8) + k5*pow(r,10) + k6*pow(r,12));
//    double y_corr_r = (double)ori_cen_y*(1.0 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6) + k4*pow(r,8) + k5*pow(r,10) + k6*pow(r,12));

////    cout << (1.0 + k1*pow(r,2) + k2*pow(r,4) + k3*pow(r,6) + k4*pow(r,8) + k5*pow(r,10) + k6*pow(r,12)) << endl;

//    ori_cen_x = ori_cen_x + x_corr_r;
//    ori_cen_y = ori_cen_y + y_corr_r;

//    r =  sqrt(ori_cen_x*ori_cen_x + ori_cen_y*ori_cen_y)/(double)sqrt((960/2)*(960/2) + (604/2)*(604/2));

//    double ori_cen_x_norm = ori_cen_x/(960.0/2.0);
//    double ori_cen_y_norm = ori_cen_y/(604.0/2.0);

//    double x_corr = ori_cen_x_norm + (2*p1*ori_cen_x_norm*ori_cen_y_norm + p2*(r*r + 2*ori_cen_x_norm*ori_cen_x_norm));
//    double y_corr = ori_cen_y_norm + (p1*(r*r + 2*ori_cen_y_norm*ori_cen_y_norm) + 2*p2*ori_cen_x_norm*ori_cen_y_norm);

//    ori_cen_x = ori_cen_x + x_corr;
//    ori_cen_y = ori_cen_y + y_corr;

//    ui->lineEdit_corr_x->setText(QString::number(ori_cen_x + 960/2));
//    ui->lineEdit_corr_y->setText(QString::number(ori_cen_y + 604/2));

////    cout << m_calib_obj.m_dist_coeffs.at<double>(0,1) << endl;
////    double x_corr_r = ori_cen_x*(1 + m_calib_obj.m_dist_coeffs[])
///

}

void G_MAIN_WINDOW::on_pushButton_load_ipm_param_clicked()
{
    QString directory;
    directory = QFileDialog::getOpenFileName(this,"Select ipm param");

    string ipm_param_path = directory.toStdString();

    cv::FileStorage fs(ipm_param_path, cv::FileStorage::READ);
    fs["m_ipm_lambda"] >> m_ipm_lambda;
    fs.release();
}

void G_MAIN_WINDOW::on_pushButton_convert_to_ipm_pixel_clicked()
{
    int ori_x = ui->lineEdit_ori_x_ipm->text().toInt();
    int ori_y = ui->lineEdit_ori_y_ipm->text().toInt();

    double A11 = m_ipm_lambda.at<double>(0,0);
    double A12 = m_ipm_lambda.at<double>(0,1);
    double A13 = m_ipm_lambda.at<double>(0,2);

    double A21 = m_ipm_lambda.at<double>(1,0);
    double A22 = m_ipm_lambda.at<double>(1,1);
    double A23 = m_ipm_lambda.at<double>(1,2);

    double A31 = m_ipm_lambda.at<double>(2,0);
    double A32 = m_ipm_lambda.at<double>(2,1);
    double A33 = m_ipm_lambda.at<double>(2,2);

    double ipm_x = (A11*(double)ori_x + A12*(double)ori_y + A13)/(A31*(double)ori_x + A32*(double)ori_y + A33);
    double ipm_y = (A21*(double)ori_x + A22*(double)ori_y + A23)/(A31*(double)ori_x + A32*(double)ori_y + A33);

    cout << ipm_x << endl;
    cout << ipm_y << endl;
}
