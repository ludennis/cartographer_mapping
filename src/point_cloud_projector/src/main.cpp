#include <iostream>
#include <string>
#include <point_cloud_projector.h>

int main( int argc, char** argv )
{
    if (argc < 2)
    {
        printf ("Usage: point_cloud_projector [map_filename]\n");
        return -1;
    }

    ros::init(argc, argv, "point_cloud_projector");
    PointCloudProjector projector_node;
    std::string map_filename = argv[1];
    projector_node.SetMapFilename(map_filename);
    projector_node.Run();

    return 0;
}
