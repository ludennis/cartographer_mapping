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

typedef pcl::PointXYZ PointT;
#define submapXYSize 100

int main (int argc, char** argv)
{
    if (argc < 3) {
        printf("Usage: create_submap [ascii|binary] [filename]\n");
        return -1;
    }

    int gridSizeX = 200;
    int gridSizeY = 200;
    pcl::PointCloud<PointT>::Ptr inputCloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT> submap[submapXYSize][submapXYSize];

    int ret = pcl::io::loadPCDFile (argv[2], *inputCloud);
    if (ret < 0) {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return -1;
    }

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*inputCloud, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    for(unsigned int index = 0; index < inputCloud->size (); index ++)
    {
        int x = submapXYSize/2 + floor(inputCloud->points[index].x/gridSizeX);
        int y = submapXYSize/2 + floor(inputCloud->points[index].y/gridSizeY);
        submap[x][y].points.push_back(inputCloud->points[index]);
    }

    for(int x=0;x<submapXYSize;x++)
        for(int y=0;y<submapXYSize;y++)
        {
            if(submap[x][y].size()>0)
            {
                // save to PCD file
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
