#include <iostream>
#include <chrono>

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

    rosbag::Bag write_bag;
    char write_bag_filename[512];
    sprintf(write_bag_filename, "%s-velodyne-extracted", read_bag_filename);
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
            if(strcmp("cpev29", vehicle)==0)
            {
                transform.setOrigin(tf::Vector3(1.2, 0.0, 1.0));
                tf::Quaternion q;
                q.setRPY(0.005, 0.14, 0.0);
                transform.setRotation(q);
            } else if(strcmp("cpev9", vehicle)==0)
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
        "threshold (%f, %f)", write_bag_filename, height_low_pass,
        height_high_pass, intensity_low_pass, intensity_high_pass);

    return 0;
}
