#include <iostream>
#include <string>

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
    std::string map_filename = argv[1];

    PointCloudTPtr map_cloud_ptr (new PointCloudT);
    pcl::io::loadPCDFile<PointT> (map_filename, *map_cloud_ptr);

    for (size_t i = 0; i < map_cloud_ptr->points.size(); ++i)
        map_cloud_ptr->points[i].z = 0;

    std::string new_map_filename = map_filename + "_2d";
    pcl::io::savePCDFileBinary(new_map_filename, *map_cloud_ptr);

    return 0;
}
