/*
Create submap from large map
Chun-Te, Dennis
*/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/program_options.hpp>
#include <boost/multi_array.hpp>

#include <ros/ros.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudTPtr;
typedef boost::multi_array<PointCloudT, 2> PointCloudArray2D;
namespace boost_po = boost::program_options;

std::string file_format;
int submap_size;
std::string map_filename;

int main (int argc, char** argv)
{
    boost_po::options_description description{"Allowed options"};
    description.add_options()
        ("help", "get help message")
        ("file-format", boost_po::value<std::string>()->default_value("binary"),
            "file format, either 'ascii' or 'binary'")
        ("submap-size", boost_po::value<int>()->default_value(50),
            "size of each submap")
        ("map-filename", boost_po::value<std::string>(), "map filename");

    boost_po::variables_map var_map;
    boost_po::store(boost_po::parse_command_line(argc, argv, description), var_map);
    boost_po::notify(var_map);

    if(var_map.count("help"))
    {
        ROS_INFO_STREAM(std::endl << description);
        return 1;
    }
    if(var_map.count("file-format"))
    {
        file_format = var_map["file-format"].as<std::string>();
        ROS_INFO_STREAM("Writing submaps with file format: " << file_format);
    }
    if(var_map.count("submap-size"))
    {
        submap_size = var_map["submap-size"].as<int>();
        ROS_INFO_STREAM("Writing submaps with each submap size: " << submap_size);
    }
    if(var_map.count("map-filename"))
    {
        map_filename = var_map["map-filename"].as<std::string>();
        ROS_INFO_STREAM("Loading map file: " << map_filename);
    } else
    {
        ROS_ERROR("No map file given, exiting");
        return -1;
    }

    PointCloudTPtr inputCloud (new PointCloudT);
    if(pcl::io::loadPCDFile(map_filename, *inputCloud) == -1)
    {
        ROS_ERROR_STREAM("Couldn't read file: " << map_filename);
        return -1;
    }

    PointT map_min_point, map_max_point;
    pcl::getMinMax3D (*inputCloud, map_min_point, map_max_point);
    ROS_INFO_STREAM("" << "Max x: " << map_max_point.x);
    ROS_INFO_STREAM("" << "Max y: " << map_max_point.y);
    ROS_INFO_STREAM("" << "Max z: " << map_max_point.z);
    ROS_INFO_STREAM("" << "Min x: " << map_min_point.x);
    ROS_INFO_STREAM("" << "Min y: " << map_min_point.y);
    ROS_INFO_STREAM("" << "Min z: " << map_min_point.z);

    const int x_grid_size = (int) ceil((map_max_point.x - map_min_point.x) / submap_size);
    const int y_grid_size = (int) ceil((map_max_point.y - map_min_point.y) / submap_size);

    PointCloudArray2D submaps(boost::extents[x_grid_size][y_grid_size]);

    for(unsigned int index = 0; index < inputCloud->size(); ++index)
    {
        const int x_grid =
            (int) floor( (inputCloud->points[index].x - map_min_point.x) / submap_size);
        const int y_grid =
            (int) floor( (inputCloud->points[index].y - map_min_point.y) / submap_size);
        submaps[x_grid][y_grid].points.push_back(inputCloud->points[index]);
    }


    for(int x = 0; x < x_grid_size; ++x)
    {
        for(int y = 0; y < y_grid_size; ++y)
        {
            if(submaps[x][y].size() > 0)
            {
                submaps[x][y].width = (int) submaps[x][y].points.size();
                submaps[x][y].height = 1;
                ROS_INFO("submaps[%d][%d]: points.size() = %ld, width = %d, "
                    "height = %d, is_dense = %d", x, y, submaps[x][y].points.size(),
                    submaps[x][y].width, submaps[x][y].height, submaps[x][y].is_dense);
                std::string submap_filename =
                    "submap_" + std::to_string(x) + "_" + std::to_string(y) + ".pcd";

                if(file_format.compare("ascii") == 0)
                {
                    if(pcl::io::savePCDFileASCII(submap_filename, submaps[x][y]) == -1)
                        ROS_ERROR_STREAM("Failed saving: " << submap_filename);
                }
                else
                {
                    if(pcl::io::savePCDFileBinary(submap_filename, submaps[x][y]) == -1)
                        ROS_ERROR_STREAM("Failed saving: " << submap_filename);
                }
            }
        }
    }
    return 0;
}
