/*
	Downsamplerer to filter input point cloud(s) with voxel grid filter
*/

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
	for (int i = 0; i < argc; ++i)
		printf("argv[%i]: %s.\n", i, argv[i]);

	if (argc < 3)
		printf("Usage: point_cloud_filterer [voxel_leaf_size] [pcd_filename1] [pcd_filename2] ...\n");

	float voxel_leaf_size = atof(argv[1]);

	for (int i = 2 ; i < argc; ++i)
	{
		printf("Voxel leaf size is set to: %f\n", voxel_leaf_size);
	
		// load pcd file
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[i], *cloud_source) == -1)
		{
			PCL_ERROR ("Couldn't read file %s.\n", argv[i]);
			return -1;
		}

		printf("Filtering %s with %i number of points ... \n", argv[i], cloud_source->width *
																		cloud_source->height); 

		// downsample with voxel grid filter
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
		vg_filter.setInputCloud(cloud_source);
		vg_filter.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
		vg_filter.filter (*cloud_filtered);

		printf("Filtered %s down to %i number of points.\n", argv[i], cloud_filtered->width *
																	  cloud_filtered->height);
		
		// write to new file
		char filtered_filename [512];
		sprintf(filtered_filename, "filtered_%s", argv[i]);
		pcl::io::savePCDFileASCII (filtered_filename, *cloud_filtered);
		
		printf("Written filtered point cloud to %s\n", filtered_filename);
	}

	
	
}
