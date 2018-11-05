#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

PointCloudTPtr map_cloud_ptr (new PointCloudT);

void pointPickingOccurred (const pcl::visualization::PointPickingEvent &event)
{
    PointT point_clicked (map_cloud_ptr->points[event.getPointIndex()]);
    ROS_INFO("Point index %i at (x=%f, y=%f, z=%f, intensity=%f) was clicked.\n",
        event.getPointIndex(), point_clicked.x,	point_clicked.y, point_clicked.z,
        point_clicked.intensity);
}

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        ROS_ERROR("Usage: point_cloud_viewer [opacity] [map_filename_1] [map_filename_2] [...]");
        return -1;
    }
    float opacity = atof(argv[1]);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->registerPointPickingCallback (pointPickingOccurred);

    for (int i = 2; i < argc; ++i)
    {
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(argv[i], *cloud);

        *map_cloud_ptr += *cloud;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity(map_cloud_ptr, "intensity");
    viewer->addPointCloud<PointT> (map_cloud_ptr, intensity, "map_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,opacity, "map_cloud");

    while(!viewer->wasStopped())
    {
        viewer->spin();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
