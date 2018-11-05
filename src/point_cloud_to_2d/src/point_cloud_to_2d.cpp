#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;
namespace boost_po = boost::program_options;

std::string map_filename;

int main(int argc, char** argv)
{
    boost_po::options_description descriptions{"Allowed options"};
    descriptions.add_options()
        ("help", "produce help messages")
        ("map-filename", boost_po::value<std::string>(), "pcd filename");

    boost_po::variables_map var_map;
    boost_po::store(boost_po::parse_command_line(argc, argv, descriptions), var_map);
    boost_po::notify(var_map);

    if(var_map.count("help"))
    {
        ROS_INFO_STREAM(std::endl << descriptions);
        return 1;
    }

    if(var_map.count("map-filename"))
    {
        map_filename = var_map["map-filename"].as<std::string>();
        ROS_INFO_STREAM("Reading pcd file: " << map_filename);
    }
    else
    {
        ROS_ERROR("No map file given, exiting");
        return -1;
    }

    PointCloudTPtr map_cloud_ptr (new PointCloudT);
    pcl::io::loadPCDFile<PointT> (map_filename, *map_cloud_ptr);

    for (size_t i = 0; i < map_cloud_ptr->points.size(); ++i)
        map_cloud_ptr->points[i].z = 0;

    std::string new_map_filename = map_filename + "_2d";
    pcl::io::savePCDFileBinary(new_map_filename, *map_cloud_ptr);

    return 0;
}
