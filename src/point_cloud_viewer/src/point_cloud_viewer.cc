/*
	Views each input point cloud and merge them with ICP matching
		1. select 2 points from each point cloud (with shift + left-click) that has overlap
		2. preview all point clouds before performing ICP matching
		3. perform ICP matching
		4. Done
*/

#include <iostream>
#include <vector>

#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>

// global variables
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_pointers;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> quadrilaterals;

// callback function to output clicked point's xyz coordinate
void pointPickingOccurred (const pcl::visualization::PointPickingEvent &event)
{
	pcl::PointXYZ point_clicked;
	event.getPoint(point_clicked.x, point_clicked.y, point_clicked.z);
	quadrilaterals.back()->push_back(point_clicked);
	printf("Point index %i at (%f, %f, %f) was clicked.\n", event.getPointIndex(), point_clicked.x,
																				   point_clicked.y,
																				   point_clicked.z);
}

// extract points from a source cloud within a quadrilateral
pcl::PointCloud<pcl::PointXYZ> getPointsWithin (const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr quad)
{
	// stores all min max xyz in two points
	pcl::PointXYZ min_bound, max_bound;
	pcl::getMinMax3D(*quad, min_bound, max_bound);

	// run source through a pass-through filter to extract points within quad
	pcl::PassThrough<pcl::PointXYZ> pass_filter;
	pass_filter.setInputCloud (source);
	
	pass_filter.setFilterFieldName ("x");
	pass_filter.setFilterLimits (min_bound.x, max_bound.x);
	pass_filter.filter (*source);

	pass_filter.setFilterFieldName ("y");
	pass_filter.setFilterLimits (min_bound.y, max_bound.y);
	pass_filter.filter (*source);
	
	return *source;
}

int main(int argc, char** argv)
{
	for (int i = 0; i < argc; ++i)
	{
		printf("argv[%i]: %s\n", i, argv[i]);
	}

	// creates PCL visualizer to view the point cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);

	// register viewer with mouse events and point clicking events
	viewer->registerPointPickingCallback (pointPickingOccurred);

	for (int i = 1; i < argc; ++i)
	{
		// clear all previously loaded point clouds
		viewer->removeAllPointClouds();

		// load point cloud file and add to vector
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *cloud);
		cloud_pointers.push_back(cloud);

		// add point cloud into viewer
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, rand() %255,
																							rand() %255,
																							rand() % 255);
		viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, argv[i]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, argv[i]);
		viewer->initCameraParameters ();

		// declare point pair class to be stored from clicking
		pcl::PointCloud<pcl::PointXYZ>::Ptr quad (new pcl::PointCloud<pcl::PointXYZ>);
		quadrilaterals.push_back(quad);

		printf("quadrilaterals size: %ld.\n", quadrilaterals.size());

		// select 4 points and use points within for icp matching
		while (quadrilaterals.back()->size() < 4)
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}

		printf("Added '%s' into viewer.\n", argv[i]);
	}

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}
