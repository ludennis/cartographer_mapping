/*
    Package to merge multiple point clouds (.pcd files) into one
*/

#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h> // for PointCloud<pcl::PointXYZ>::Ptr
#include <pcl/registration/icp.h> //for pcl::IterativeClosestPoint
#include <pcl/filters/voxel_grid.h>

int main(int argc, char* argv[])
{
    std::clock_t begin;
    std::clock_t end;

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

    // downsample both source and target clouds with voxel grid filter
    printf("Downsampling point clouds ... \n");
    begin = std::clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid_filter.setInputCloud(cloud_source);
    voxel_grid_filter.filter(*source_filtered);
    voxel_grid_filter.setInputCloud(cloud_target);
    voxel_grid_filter.filter(*target_filtered);

    end = std::clock();
    printf("Time elapsed for downsampling: %f\n", double(end - begin) / CLOCKS_PER_SEC);

    // run ICP over point clouds
    printf("Aligning source with target using ICP ... \n");

    begin = std::clock();

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_filtered);
    icp.setInputTarget(target_filtered);
    pcl::PointCloud<pcl::PointXYZ> cloud_merged;

    icp.align(cloud_merged);

    end = std::clock();


    printf("ICP has converged: %i with score: %f\n", icp.hasConverged(), icp.getFitnessScore());
    printf("Time elapsed for ICP: %f\n", double(end - begin) / CLOCKS_PER_SEC);
    printf("Final transformation: \n");
    std::cout << icp.getFinalTransformation() << std::endl;

    // save in file
    begin = std::clock();

    pcl::io::savePCDFileBinary(argv[3], cloud_merged);
    end = std::clock();

    printf("Written point cloud data to file: '%s'\n", argv[3]);
    printf("Time elapsed for writing to file: %f\n", double(end - begin) / CLOCKS_PER_SEC);

    return 0;
}
