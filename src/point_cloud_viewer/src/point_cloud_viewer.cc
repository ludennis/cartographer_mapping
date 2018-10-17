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
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZRGBA PointT;

// callback function to output clicked point's xyz coordinate
void pointPickingOccurred (const pcl::visualization::PointPickingEvent &event)
{
	PointT point_clicked;
	event.getPoint(point_clicked.x, point_clicked.y, point_clicked.z);
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

	float opacity = atof(argv[1]);

	// creates PCL visualizer to view the point cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);

	// register viewer with mouse events and point clicking events
	viewer->registerPointPickingCallback (pointPickingOccurred);

	for (int i = 2; i < argc; ++i)
	{
		// load point cloud file and add to vector
		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		pcl::io::loadPCDFile<PointT>(argv[i], *cloud);

		// add point cloud into viewer
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color_white(cloud,255,255,255);
		viewer->addPointCloud<PointT> (cloud, color_white, argv[i]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, argv[i]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,opacity, argv[i]);
		viewer->initCameraParameters ();
	}

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}
