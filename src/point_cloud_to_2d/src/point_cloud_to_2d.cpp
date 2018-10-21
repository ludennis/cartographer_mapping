#include <iostream>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Usage: point_cloud_to_2d [pcd_filename]");
        return -1;
    }
    char map_filename [512];
    strcpy(map_filename, argv[1]);

    PointCloudTPtr map_cloud_ptr (new PointCloudT);
    pcl::io::loadPCDFile<PointT> (map_filename, *map_cloud_ptr);

    for (size_t i = 0; i < map_cloud_ptr->points.size(); ++i)
        map_cloud_ptr->points[i].z = 0;

    char new_map_filename [512];
    sprintf(new_map_filename, "%s_2d", map_filename);
    pcl::io::savePCDFileBinary(new_map_filename, *map_cloud_ptr);

    return 0;
}
