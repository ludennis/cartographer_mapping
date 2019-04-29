#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        ROS_ERROR("Usage: point_cloud_transformer [map_filename]");
        return -1;
    }
    char map_filename[512];
    strcpy(map_filename, argv[1]);

    PointCloudTPtr map_cloud_ptr (new PointCloudT);
    pcl::io::loadPCDFile(map_filename, *map_cloud_ptr);

    Eigen::Matrix4f transformation;
    transformation <<      -0.81739,    -0.576085,  0.000102261,      9.05301,
    0.576085 ,    -0.81739, -3.23579e-05  ,   -11.3278,
 0.000102228,  3.24631e-05      ,      1  ,   -8.29385,
           0,            0   ,         0  ,          1;


    pcl::transformPointCloud(*map_cloud_ptr, *map_cloud_ptr, transformation);

    char new_map_filename[512];
    sprintf(new_map_filename, "transformed_%s", map_filename);
    pcl::io::savePCDFileBinary(new_map_filename, *map_cloud_ptr);

    return 0;
}
