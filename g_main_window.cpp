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

    connect(c_t_sceneupdate,SIGNAL(SIG_C_T_SCENEUPDATE_2_MAIN()),this,SLOT(SLOT_C_T_SCENEUPDATE_2_MAIN()));
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
                string token;
                vector<string> tokens;
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

// ------------------------------------------------------------------------
