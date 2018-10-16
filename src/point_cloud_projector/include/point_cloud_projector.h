#ifndef POINT_CLOUD_PROJECTOR_H
#define POINT_CLOUD_PROJECTOR_H

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointT>::Ptr PointCloudTPtr;
typedef pcl::registration::TransformationEstimationSVD<PointT,PointT> TransSVDT;

struct GPSCoord {
    float latitude;
    float longitude;
    float altitude;
};

struct CartesianCoord {
    float x;
    float y;
    float z;
};

class PointCloudProjector
{
public:
    PointCloudProjector(){}
    void SetMapFilename(const std::string& map_filename);
    void Run();
private:
    int mGPSFixQuality;
    PointCloudT mPredictPosesCloud;
    PointCloudT mGPSPosesCloud;
    ros::NodeHandle mNodeHandle;
    ros::Subscriber mPredictPoseSub;
    ros::Subscriber mNmeaSub;
    ros::Subscriber mFixSub;
    TransSVDT mTransEstimator;
    Eigen::Matrix4f mRigidTransformation;
    std::string mMapFilename;

    void PredictPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void NMEACallback(const nmea_msgs::Sentence::ConstPtr& msg);
    void FixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    CartesianCoord GPSToCartesian(const GPSCoord);
};

#endif // POINT_CLOUD_PROJECTOR_H
