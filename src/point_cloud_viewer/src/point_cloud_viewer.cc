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

// global variables
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_pointers;

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
	}

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}
