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
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

class Line
{
public:
	float slope, intersect;
	Line(){}
	Line(float slope, float intersect)	
	{
		this->slope = slope;
		this->intersect = intersect;
	}
	Line(pcl::PointXYZ pt1, pcl::PointXYZ pt2)
	{
		this->slope = (pt1.y - pt2.y) / (pt1.x - pt2.x);
		this->intersect = pt1.y - this->slope * pt1.x;
	}
};

// global variables
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_pointers;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> quadrilaterals;
bool enterKeyPressed = false;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event)
{
	if ( event.getKeyCode() == 13)
	{
		enterKeyPressed = true;
		printf("Enter Key pressed!\n");
	}
}

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
	if (argc < 4)
	{
		printf("Usage: point_cloud_merger [icp|ndt] [source_cloud] [target_cloud1] [target_cloud2] ...\n");
	}
	if (strcmp("icp", argv[1]) != 0 and strcmp("ndt", argv[1]) !=0)
	{
		printf("Error: argv[1] must be 'icp' or 'ndt'.\n");
		return -1;
	}
	for (int i = 0; i < argc; ++i)
	{
		printf("argv[%i]: %s\n", i, argv[i]);
	}

	// creates PCL visualizer to view the point cloud
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);

	// register viewer with mouse events and point clicking events
	viewer->registerPointPickingCallback (pointPickingOccurred);

	viewer->registerKeyboardCallback (keyboardEventOccurred);
	for (int i = 2; i < argc; ++i)
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

	// calculate how to map all into the same coordinate with quadrilaterals vector
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// wait until enter key pressed before matching
	enterKeyPressed = false;
	printf("Press Enter Key to start matching ...\n");
	while(!enterKeyPressed)
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr icp_target (new pcl::PointCloud<pcl::PointXYZ>);

	*icp_source = getPointsWithin(cloud_pointers.front(), quadrilaterals.front());
	*icp_target = getPointsWithin(cloud_pointers.back(), quadrilaterals.back());	

	icp.setInputSource(icp_source);
	icp.setInputTarget(icp_target);

	pcl::PointCloud<pcl::PointXYZ> result;	

	icp.setMaximumIterations(1000);
	icp.setTransformationEpsilon(1e-9);
	icp.align(result);

	std::cout << "ICP has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	// apply this transformation to source 
	printf("Applying transformation matrix to source cloud ... \n");
	Eigen::Matrix<float, 4, 4> trans_matrix = icp.getFinalTransformation();
	pcl::transformPointCloud (*cloud_pointers.front() , *cloud_pointers.front(), trans_matrix);

	// display all loaded point clouds after transformation
	viewer->removeAllPointClouds();
	for (size_t i = 0; i < cloud_pointers.size() ; ++i)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_pointers[i],
			rand() % 255, rand() % 255, rand() % 255);
		viewer->addPointCloud<pcl::PointXYZ> (cloud_pointers[i], single_color, argv[i+1]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, argv[i+1]);
		viewer->initCameraParameters();
	}

	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}
