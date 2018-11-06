#include <chrono>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

#include <boost/program_options.hpp>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;
namespace boost_po = boost::program_options;

std::string vehicle;
double height_low_pass, height_high_pass;
double intensity_low_pass, intensity_high_pass;
std::string read_bag_filename;

int main (int argc, char** argv)
{
    boost_po::options_description description{"Allowed options"};
    description.add_options()
        ("help", "get help message")
        ("vehicle", boost_po::value<std::string>()->default_value("cpev9"),
            "set vehicle to cpev9|cpev27|cpev29|s3")
        ("height-low-pass", boost_po::value<double>()->default_value(-200.0),
            "height lower than this will be filtered out")
        ("height-high-pass", boost_po::value<double>()->default_value(200.0),
            "height higher than this will be filtered out")
        ("intensity-low-pass", boost_po::value<int>()->default_value(0),
            "intensity lower than this will be filtered out")
        ("intensity-high-pass", boost_po::value<int>()->default_value(256),
            "intensity higher than this will be filtered out")
        ("read-bag-filename", boost_po::value<std::string>(),
            "bag filename to be read");

    boost_po::variables_map var_map;
    boost_po::store(boost_po::parse_command_line(argc, argv, description), var_map);
    boost_po::notify(var_map);

    if(var_map.count("help"))
    {
        ROS_INFO_STREAM(std::endl << description);
        return 1;
    }
    if(var_map.count("vehicle"))
    {
        vehicle = var_map["vehicle"].as<std::string>();
        ROS_INFO_STREAM("Vehicle is set to: " << vehicle);
    }
    if(var_map.count("height-low-pass"))
    {
        height_low_pass = var_map["height-low-pass"].as<double>();
        ROS_INFO_STREAM("Height low pass set to: " << height_low_pass);
    }
    if(var_map.count("height-high-pass"))
    {
        height_high_pass = var_map["height-high-pass"].as<double>();
        ROS_INFO_STREAM("Height high pass set to: " << height_high_pass);
    }
    if(var_map.count("intensity-low-pass"))
    {
        intensity_low_pass = var_map["intensity-low-pass"].as<int>();
        ROS_INFO_STREAM("Intensity low pass set to: " << intensity_low_pass);
    }
    if(var_map.count("intensity-high-pass"))
    {
        intensity_high_pass = var_map["intensity-high-pass"].as<int>();
        ROS_INFO_STREAM("Intensity high pass set to: " << intensity_high_pass);
    }
    if(var_map.count("read-bag-filename"))
    {
        read_bag_filename = var_map["read-bag-filename"].as<std::string>();
        ROS_INFO_STREAM("Read bag filename set to: " << read_bag_filename);
    } else
    {
        ROS_ERROR("Read bag filename was not set, exiting");
        return -1;
    }

    rosbag::Bag read_bag;
    read_bag.open(read_bag_filename, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/velodyne_points"));
    topics.push_back(std::string("/fix"));
    topics.push_back(std::string("/imu/data"));

    rosbag::Bag write_bag;
    std::string write_bag_filename = read_bag_filename + "-velodyne-extracted";
    write_bag.open(write_bag_filename, rosbag::bagmode::Write);

    pcl::PassThrough<PointT> pass_filter;

    auto start = std::chrono::system_clock::now();

    PointCloudTPtr total_filtered_cloud_ptr (new PointCloudT);

    for(rosbag::MessageInstance const m: rosbag::View(
        read_bag, rosbag::TopicQuery(topics)))
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

            tf::Transform transform, transform_inverse;
            if(vehicle.compare("cpev29") == 0)
            {
                transform.setOrigin(tf::Vector3(1.2, 0.0, 1.0));
                tf::Quaternion q;
                q.setRPY(0.005, 0.14, 0.0);
                transform.setRotation(q);
            } else if(vehicle.compare("cpev9") == 0)
            {
                transform.setOrigin(tf::Vector3(0.0, 0.0, 1.5));
                tf::Quaternion q;
                q.setRPY(0.0, 0.02, 0.0);
                transform.setRotation(q);
            }
            pcl_ros::transformPointCloud(*cloud_ptr, *cloud_ptr, transform);

            ROS_INFO ("Number of points before filtering: %ld points",
                cloud_ptr->points.size());

            pass_filter.setInputCloud(cloud_ptr);
            pass_filter.setFilterFieldName("intensity");
            pass_filter.setFilterLimits(intensity_low_pass, intensity_high_pass);
            pass_filter.filter(*cloud_ptr);

            pass_filter.setInputCloud(cloud_ptr);
            pass_filter.setFilterFieldName("z");
            pass_filter.setFilterLimits(height_low_pass, height_high_pass);
            pass_filter.filter(*cloud_ptr);

            ROS_INFO ("Number of points after filtering: %ld points",
                cloud_ptr->points.size());

            transform_inverse = transform.inverse();
            pcl_ros::transformPointCloud (*cloud_ptr, *cloud_ptr, transform_inverse);
            sensor_msgs::PointCloud2 filtered_pc2_msg;
            pcl::toROSMsg (*cloud_ptr, filtered_pc2_msg);

            *total_filtered_cloud_ptr += *cloud_ptr;
            write_bag.write("/velodyne_points", msg_time, filtered_pc2_msg);
        }

        if ( fix_msg != NULL )
            write_bag.write("/fix", msg_time, *fix_msg);
        if ( imu_msg != NULL )
            write_bag.write("/imu/data", msg_time, *imu_msg);
    }

    size_t max_intensity = 256;
    int intensity_counts[max_intensity];
    std::fill_n(intensity_counts, max_intensity, 0);

    for (size_t i = 0; i < total_filtered_cloud_ptr->points.size(); ++i)
    {
        PointT point = total_filtered_cloud_ptr->points[i];
        intensity_counts[(int)point.intensity]++;
    }

    for (size_t i = 0; i < max_intensity; ++i)
        ROS_INFO ("Intensity[%ld]: %d", i, intensity_counts[i]);

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    ROS_INFO ("Time Elapsed: %f seconds", elapsed_seconds.count());
    ROS_INFO ("Wrote to filename '%s' with height_threshold(%f, %f) and intensity "
        "threshold (%f, %f)", write_bag_filename.c_str(), height_low_pass,
        height_high_pass, intensity_low_pass, intensity_high_pass);

    return 0;
}
