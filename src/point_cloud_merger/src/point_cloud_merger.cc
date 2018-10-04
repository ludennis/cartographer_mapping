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

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_2D.h>
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
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;
float VOXEL_LEAF_SIZE = 0.4f;
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

		// filter source clouds
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
		vg_filter.setInputCloud (cloud);
		vg_filter.setLeafSize(VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE, VOXEL_LEAF_SIZE);
		vg_filter.filter (*filtered_cloud);
		filtered_clouds.push_back(filtered_cloud);

		// add point cloud into viewer
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, rand() %255,
																							rand() %255,
																							rand() % 255);
		viewer->addPointCloud<pcl::PointXYZ> (filtered_cloud, single_color, argv[i]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, argv[i]);
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
	}

	// map source cloud to target cloud with their respective quadruples
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> tf_est;

	Eigen::Matrix4f init_guess;
	tf_est.estimateRigidTransformation(*(quadruples.front()), *(quadruples.back()), init_guess);
	pcl::transformPointCloud(*(filtered_clouds.front()), *(filtered_clouds.front()), init_guess);

	// use only points within target's quadruple for matching
	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_target (new pcl::PointCloud<pcl::PointXYZ>);

	*cropped_source = getPointsWithin(filtered_clouds.front(), quadruples.back());
	*cropped_target = getPointsWithin(filtered_clouds.back(), quadruples.back());
	
	// display cropped point clouds
	viewer->removeAllPointClouds();
	for (size_t i = 0; i < filtered_clouds.size() ; ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		*cropped_cloud = getPointsWithin(filtered_clouds[i], quadruples.back());
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cropped_cloud,
			rand() % 255, rand() % 255, rand() % 255);
		viewer->addPointCloud<pcl::PointXYZ> (cropped_cloud, single_color, argv[i+1]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, argv[i+1]);
		viewer->initCameraParameters();
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

	// matching between the cropped_source and cropped_target
	Eigen::Matrix4f final_trans_matrix;
	if ( strcmp(argv[1],"icp") == 0)
	{
		// icp matching
		printf("Matching source cloud with target cloud with ICP ... \n");	
		begin_time = clock();

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		pcl::PointCloud<pcl::PointXYZ>::Ptr icp_source (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr icp_target (new pcl::PointCloud<pcl::PointXYZ>);
	
		*icp_source = *cropped_source;
		*icp_target = *cropped_target;
	
		icp.setInputSource(icp_source);
		icp.setInputTarget(icp_target);
	
		pcl::PointCloud<pcl::PointXYZ> result;	
	
		icp.setMaxCorrespondenceDistance (10.0f);
		icp.setMaximumIterations(1000);
		icp.setTransformationEpsilon(1e-9);
		icp.setEuclideanFitnessEpsilon (0.0001f);
		icp.align(result);


		end_time = clock();
		std::cout << "ICP has converged: " << icp.hasConverged() << " score: " 
				  << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
		std::cout << "Elapsed time: " << double(end_time - begin_time) / CLOCKS_PER_SEC << std::endl;
	
		// applying icp's transformation to source cloud
		final_trans_matrix = icp.getFinalTransformation();
	} else if ( strcmp(argv[1],"ndt") == 0 )
	{
		// ndt matching
		printf("Matching source cloud with target cloud with NDT ... \n");
		pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_source (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_target (new pcl::PointCloud<pcl::PointXYZ>);

		// maps the cropped area to the full filtered target cloud
		*ndt_source = *cropped_source;
		*ndt_target = *cropped_target;

		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
		vg_filter.setInputCloud (ndt_source);
		vg_filter.setLeafSize(2.0f, 2.0f, 2.0f);
		vg_filter.filter (*ndt_source);

		ndt.setInputSource(ndt_source);
		ndt.setInputTarget(ndt_target);
		
		// initial ndt parameters
		float trans_epi = 0.005f;
		float step_size = 0.05f;
		int max_iteration = 200;
		int num_rematch = 0;
		float ndt_trans_prob = 2.5f;
		init_guess = Eigen::Matrix4f::Identity();
	
		std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;
		matching_start = std::chrono::system_clock::now();

		do 
		{			
			// ndt parameters
			ndt.setTransformationEpsilon (trans_epi);
			ndt.setStepSize (step_size);
			ndt.setResolution (1.0f);
			ndt.setOulierRatio(0.2f);
			ndt.setMaximumIterations (max_iteration);


			pcl::PointCloud<pcl::PointXYZ> result;
			ndt.align (result, init_guess);
			printf ("Number of rematch: %d\n", num_rematch++);
			printf ("Normal Distributions Transform has converged: %d, score: %f, iterations: %d, \
					 outlier ratio: %f, trans prob: %f, resolution: %f, step size: %f, epsilon: %f\n", 
					 ndt.hasConverged(), ndt.getFitnessScore(), ndt.getFinalNumIteration(), 
					 ndt.getOulierRatio(), ndt.getTransformationProbability(), ndt.getResolution(),
					 step_size, trans_epi);
			
			// visualize

			// update ndt parameters
			//trans_epi /= 1.1f;
			//step_size /= 1.1f;

			// randomize init guess matrix within translation range < 1m and rotation angel < 5 degrees
			float x_rotation_deg = (float) (rand() % 40000 - 20000) / 10000.0f; // +- 2.0000 degree
			float y_rotation_deg = (float) (rand() % 40000 - 20000) / 10000.0f; // +- 2.0000 degree
			float z_translation = (float) (rand() % 20000 - 10000) / 1000.0f; // +- 10.0000 m
			printf ("Randomized init matrix with x_rotation: %f, y_rotation: %f, z_translation: %f\n",
				x_rotation_deg, y_rotation_deg, z_translation);
			Eigen::AngleAxisf init_rotation_x ( x_rotation_deg * M_PI/ 180.0f , Eigen::Vector3f::UnitX());
			Eigen::AngleAxisf init_rotation_y ( y_rotation_deg * M_PI/ 180.0f , Eigen::Vector3f::UnitY());
			Eigen::Translation3f init_translation ( 0, 0, z_translation ) ;
			init_guess = (init_translation * init_rotation_x * init_rotation_y).matrix ();
			if (num_rematch % 50 == 0)
				ndt_trans_prob += 0.5f;

			final_trans_matrix = ndt.getFinalTransformation();
		} 
		while (ndt.getTransformationProbability () > ndt_trans_prob or ndt.getFitnessScore() > 1.0f);

		// apply final transformation		
		pcl::transformPointCloud (*(filtered_clouds.front()), *(filtered_clouds.front()), final_trans_matrix);

	}

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
