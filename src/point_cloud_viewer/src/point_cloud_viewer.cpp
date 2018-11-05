#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;
namespace boost_po = boost::program_options;

PointCloudTPtr map_cloud_ptr (new PointCloudT);

std::vector<std::string> map_filenames;
double opacity;

void pointPickingOccurred (const pcl::visualization::PointPickingEvent &event)
{
    PointT point_clicked (map_cloud_ptr->points[event.getPointIndex()]);
    ROS_INFO("Point index %i at (x=%f, y=%f, z=%f, intensity=%f) was clicked.\n",
        event.getPointIndex(), point_clicked.x,	point_clicked.y, point_clicked.z,
        point_clicked.intensity);
}

int main(int argc, char** argv)
{
    boost_po::options_description descriptions{"Allowed options"};
    descriptions.add_options()
        ("help", "produce help message")
        ("opacity", boost_po::value<double>()->default_value(1.0),
            "set opacity(alpha) value")
        ("map-filenames", boost_po::value<std::vector<std::string>>()->multitoken(),
            "map filename(s)");

    boost_po::variables_map var_map;
    boost_po::store(boost_po::parse_command_line(argc, argv, descriptions), var_map);
    boost_po::notify(var_map);

    if(var_map.count("help"))
    {
        ROS_INFO_STREAM(std::endl << descriptions);
        return 1;
    }

    if (var_map.count("opacity"))
    {
        ROS_INFO_STREAM("Opacity set to: " << var_map["opacity"].as<double>());
        opacity = var_map["opacity"].as<double>();
    }

    if (var_map.count("map-filenames"))
    {
        map_filenames = var_map["map-filenames"].as<std::vector<std::string>>();

        for(auto itr = map_filenames.begin(); itr != map_filenames.end(); ++itr)
        {
            ROS_INFO_STREAM("Map filenames: " << *itr);
        }
    }
    else
    {
        ROS_ERROR("No map files given, exiting");
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->registerPointPickingCallback (pointPickingOccurred);

    for (auto itr = map_filenames.begin(); itr != map_filenames.end(); ++itr)
    {
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile<PointT>(*itr, *cloud);

        *map_cloud_ptr += *cloud;
    }

    pcl::visualization::PointCloudColorHandlerGenericField<PointT> intensity(map_cloud_ptr, "intensity");
    viewer->addPointCloud<PointT> (map_cloud_ptr, intensity, "map_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,opacity, "map_cloud");

    while(!viewer->wasStopped())
    {
        viewer->spin();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
