#include <iostream>

int main (int argc, char** argv)
{
    if (argc < 6)
    {
        ROS_ERROR ("Usage: velodyne_extractor [vehicle] [height_low_pass] [height_high_pass] \
            [intensity_low_pass] [intensity_high_pass] [bag_filename]");
        return -1;
    }
    char read_bag_filename[512];
    char vehicle[512];
    strcpy(vehicle, argv[1]);
    float height_low_pass(atof(argv[2]));
    float height_high_pass(atof(argv[3]));
    float intensity_low_pass(atof(argv[4]));
    float intensity_high_pass(atof(argv[5]));
    strcpy(read_bag_filename, argv[6]);

    return 0;
}
