/*
	Views each input point cloud and merge them with ICP matching
		1. select 2 points from each point cloud (with shift + left-click) that has overlap
		2. preview all point clouds before performing ICP matching
		3. perform ICP matching
		4. Done
*/

#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>

// global variables
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_pointers;

int main(int argc, char** argv)
{
	for (int i = 0; i < argc; ++i)
	{
		printf("argv[%i]: %s\n", i, argv[i]);
	}

	for (int i = 1; i < argc; ++i)
	{
		// load point cloud file and add to vector
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *cloud);
		cloud_pointers.push_back(cloud);
	}

	return 0;
}
