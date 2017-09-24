#include "c_3d_viewer.h"

void C_3D_VIEWER::init()
{
    cloud.reset(new PointCloudT);
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer",false));
}
