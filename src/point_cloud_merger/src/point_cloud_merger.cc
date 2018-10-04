/*
	Views each input point cloud and merge them with ICP matching
		1. select 2 points from each point cloud (with shift + left-click) that has overlap
		2. preview all point clouds before performing ICP matching
		3. perform ICP matching
		4. Done
*/

#include <chrono>
#include <ctime>
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
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> source_clouds;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> quadruples;
float VOXEL_LEAF_SIZE = 0.4f;
bool enterKeyPressed = false;
clock_t begin_time, end_time;

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
	quadruples.back()->push_back(point_clicked);
	printf("Point index %i at (%f, %f, %f) was clicked.\n", event.getPointIndex(), point_clicked.x,
																				   point_clicked.y,
																				   point_clicked.z);
}

pcl::PointCloud<pcl::PointXYZ> getPointsWithin (const pcl::PointCloud<pcl::PointXYZ>::Ptr source,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr quadruple)
{
	// get the 4 lines from the 4 points in quadruple (tl = top left, br = bottom right)
	Line line_tl_tr( quadruple->points[0] , quadruple->points[1] );
	Line line_tr_br( quadruple->points[1] , quadruple->points[2] );
	Line line_br_bl( quadruple->points[2] , quadruple->points[3] );
	Line line_bl_tl( quadruple->points[3] , quadruple->points[0] );

	for ( size_t i = 0; i < quadruple->points.size(); ++i)
	{
		printf("quadruple->points[%ld]: %f, %f, %f\n", i, quadruple->points[i].x,
			quadruple->points[i].y, quadruple->points[i].z);
	}
	
	printf ("line_tl_tr: slope = %f, intersect = %f\n", line_tl_tr.slope, line_tl_tr.intersect);

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_extracted (new pcl::PointCloud<pcl::PointXYZ>);
	for ( size_t i = 0; i < source->points.size(); ++i)
	{
		float x = source->points[i].x, y = source->points[i].y;
		
		if ((	y < line_tl_tr.slope * x + line_tl_tr.intersect and
				y > line_br_bl.slope * x + line_br_bl.intersect and
				y < line_tr_br.slope * x + line_tr_br.intersect and
				y > line_bl_tl.slope * x + line_bl_tl.intersect)
			or
			(	y >= line_tl_tr.slope * x + line_tl_tr.intersect and
				y <= line_br_bl.slope * x + line_br_bl.intersect and
				y >= line_tr_br.slope * x + line_tr_br.intersect and 
				y <= line_bl_tl.slope * x + line_bl_tl.intersect))
		{
			source_extracted->points.push_back(source->points[i]);
		}	
	}

	printf ("source->points.size() = %ld, source_extracted->points.size() = %ld\n", 
		source->points.size(), source_extracted->points.size());

	return *source_extracted;
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

	// register viewer with keyboard events and point clicking events
	viewer->registerPointPickingCallback (pointPickingOccurred);
	viewer->registerKeyboardCallback (keyboardEventOccurred);
	for (int i = 2; i < argc; ++i)
	{
		// clear all previously loaded point clouds
		viewer->removeAllPointClouds();

		// load point cloud file and add to vector
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *cloud);
		source_clouds.push_back(cloud);

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
		pcl::PointCloud<pcl::PointXYZ>::Ptr quadruple (new pcl::PointCloud<pcl::PointXYZ>);
		quadruples.push_back(quadruple);

		printf("quadruples.size(): %ld.\n", quadruples.size());

		// select 4 points and use points within for icp matching
		while (quadruples.back()->size() < 4)
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

		matching_end = std::chrono::system_clock::now();
		printf ("Time for matching: %f ms\n", std::chrono::duration_cast<std::chrono::microseconds>(
			matching_end - matching_start).count() / 1000.0);
	}

	// reload updated clouds
	viewer->removeAllPointClouds();
	for (size_t i = 0; i < filtered_clouds.size(); ++i)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(filtered_clouds[i],
			rand() % 255, rand() % 255, rand() % 255);
		viewer->addPointCloud<pcl::PointXYZ> (filtered_clouds[i], single_color, argv[i+1]);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, argv[i+1]);
		viewer->initCameraParameters();
	}

	// merge source and target clouds into one cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < filtered_clouds.size(); ++i)
	{
		*merged_cloud += *filtered_clouds[i];
	}
	printf("Written merged point clouds into file 'merged.pcd'.\n");
	pcl::io::savePCDFileASCII ("merged.pcd", *merged_cloud);


	while(!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	// combine all point clouds and write them into one merged point cloud file
	


	return 0;
}
