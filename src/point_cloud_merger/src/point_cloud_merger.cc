/*
    Package to merge multiple point clouds (.pcd files) into one
*/

#include <iostream>
#include <pcl/io/pcd_io.h>

int main(int argc, char* argv[])
{
    if (argc < 4)
    {
        printf("Usage: point_cloud_merger [source pcd filename] [target pcd filename] "
            "[output pcd filename]\n");
        return -1;
    }

    // declare point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

    //load point clouds (.pcd files) from commandline arguments
    printf("Opening pcd file: '%s' ... ", argv[1]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud_source) == -1)
    {
        PCL_ERROR ("Error: couldn't read file %s\n", argv[1]);
        return -1;
    }
    else
    {
        printf("Success! Loaded %i data points.\n", cloud_source->width * cloud_source->height);
    }

    printf("Opening pcd file: '%s' ... ", argv[2]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *cloud_target) == -1)
    {
        PCL_ERROR ("Error: couldn't read file %s\n", argv[2]);
        return -1;
    }
    else
    {
        printf("Success! Loaded %i data points.\n", cloud_target->width * cloud_target->height);
    }

    // run ICP over point clouds

    // apply transformations to a combined point clouds

    // save in file

    return 0;
}
