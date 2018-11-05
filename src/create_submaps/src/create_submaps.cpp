/*
Create submap from large map
Chun-Te
*/

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include "boost/multi_array.hpp"

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudTPtr;
typedef boost::multi_array<PointCloudT, 2> PointCloudArray2D;

int main (int argc, char** argv)
{
    if (argc < 4) {
        printf("Usage: create_submap [ascii|binary] [submap_size] [filename]\n");
        return -1;
    }
    char file_format[512];
    strcpy(file_format, argv[1]);
    const int submap_size = atoi(argv[2]);
    char filename[512];
    strcpy(filename, argv[3]);

    PointCloudTPtr inputCloud (new PointCloudT);
    if(pcl::io::loadPCDFile(filename, *inputCloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", filename);
        return -1;
    }

    PointT map_min_point, map_max_point;
    pcl::getMinMax3D (*inputCloud, map_min_point, map_max_point);
    std::cout << "Max x: " << map_max_point.x << std::endl;
    std::cout << "Max y: " << map_max_point.y << std::endl;
    std::cout << "Max z: " << map_max_point.z << std::endl;
    std::cout << "Min x: " << map_min_point.x << std::endl;
    std::cout << "Min y: " << map_min_point.y << std::endl;
    std::cout << "Min z: " << map_min_point.z << std::endl;

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
                // save to PCD file
                submaps[x][y].width = (int) submaps[x][y].points.size();
                submaps[x][y].height = 1;
                printf ("submaps[%d][%d]: points.size() = %ld, width = %d, height = %d, is_dense = %d\n",
				    x, y, submaps[x][y].points.size(), submaps[x][y].width, submaps[x][y].height,
				    submaps[x][y].is_dense);
                char filename[100];
                sprintf(filename,"./submap_%d_%d.pcd",x-submapXYSize/2,  y-submapXYSize/2);

			    if ( strcmp( argv[1], "ascii") == 0 )
			    {
                    if(pcl::io::savePCDFileASCII(filename, submaps[x][y]) == -1)
                    {
                        std::cout << "Failed saving " << filename << "." << std::endl;
                    }
                }
				else
				{
					if(pcl::io::savePCDFileBinary(filename, submaps[x][y]) == -1)
					{
						std::cout << "Failed saving " << filename << "." << std::endl;
					}
				}
            }
        }
    return 0;
}
