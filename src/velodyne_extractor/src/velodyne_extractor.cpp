#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;

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

    rosbag::Bag read_bag;
    read_bag.open(read_bag_filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));
    topics.push_back(std::string("/fix"));
    topics.push_back(std::string("/imu/data"));

    rosbag::View view(read_bag, rosbag::TopicQuery(topics));

    foreach (rosbag::MessageInstance const m, view)
    {
        ros::Time msg_time = m.getTime();
        sensor_msgs::PointCloud2::ConstPtr pc2_msg =
            m.instantiate<sensor_msgs::PointCloud2>();
        sensor_msgs::NavSatFix::ConstPtr fix_msg =
            m.instantiate<sensor_msgs::NavSatFix>();
        sensor_msgs::Imu::ConstPtr imu_msg =
            m.instantiate<sensor_msgs::Imu>();

        if ( pc2_msg != NULL)
        {
            PointCloudTPtr cloud_ptr(new PointCloudT);
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(*pc2_msg, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *cloud_ptr);
        }
    }

    return 0;
}
