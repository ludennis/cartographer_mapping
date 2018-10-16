#include <point_cloud_projector.h>
#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>

const GPSCoord gps_reference{ .latitude = 24.775084704,
    .longitude = 121.045888961, .altitude = 146.593 };

void PointCloudProjector::SetMapFilename(const std::string& map_filename)
{
    mMapFilename = map_filename;
}

CartesianCoord PointCloudProjector::GPSToCartesian(const GPSCoord gps_coord)
{
    const float a = 6378137.0;
    const float b = 6356752.3142;
    float e_squared = 1.0f - pow ( (b/a), 2);

    // location of reference point in radians
    double phi = gps_reference.latitude * M_PI / 180.0f;
    double lam = gps_reference.longitude * M_PI / 180.0f;
    float h = gps_reference.altitude;

    // localtion of data points in radians
    double delta_phi = (gps_coord.latitude * M_PI / 180.0f) - phi;
    double delta_lam = (gps_coord.longitude * M_PI / 180.0f) - lam;
    float delta_h = gps_coord.altitude - h;

    // some useful definitions
    float cos_phi = cos(phi);
    float sin_phi = sin(phi);
    float N = sqrt ( 1.0f - e_squared * pow(sin_phi, 2));

    // transformations
    double x = ( a / N + h) * cos_phi * delta_lam -
        ( (a * (1.0f - e_squared)) / pow(N, 3) + h ) * sin_phi * delta_phi * delta_lam +
        cos_phi * delta_lam * delta_h;
    double y = ( a * (1.0f - e_squared) / pow(N, 3) + h) * delta_phi +
        1.5f * cos_phi * sin_phi * a * e_squared * pow(delta_phi, 2) +
        pow(sin_phi, 2) * delta_h * delta_phi +
        0.5f * sin_phi * cos_phi * (a / N + h) * pow(delta_lam, 2);
    double z = delta_h - 0.5f * (a - 1.5f * a * e_squared * pow(cos_phi, 2) +
        0.5f * a * e_squared + h) * pow(delta_phi, 2) -
        0.5f * pow(cos_phi, 2) * (a / N - h) * pow(delta_lam, 2);

    return CartesianCoord{ .x = (float)x, .y = (float)y, .z = (float)z };
}

void PointCloudProjector::PredictPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Predict Pose: (%f, %f, %f)\n",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (mGPSFixQuality == 4)
        mPredictPosesCloud.points.push_back(
            PointT(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
}

void PointCloudProjector::NMEACallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
    std::stringstream ss(msg->sentence);
    std::string token;
    std::vector<std::string> nmea_sentence;
    while (std::getline(ss,token,','))
        nmea_sentence.push_back(token);
    if ( nmea_sentence[0].compare("$GPGGA") == 0)
        mGPSFixQuality = std::stoi(nmea_sentence[6]);
}

void PointCloudProjector::FixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ROS_INFO("NavSatFix: latitude: %f, longitude: %f, altitude: %f, fix quality: %d\n",
        msg->latitude, msg->longitude, msg->altitude, mGPSFixQuality);
    if (mGPSFixQuality == 4)
    {
        GPSCoord gps_coord { .latitude = (float)msg->latitude,
            .longitude = (float)msg->longitude, .altitude = (float)msg->altitude };
        CartesianCoord gps_translated = GPSToCartesian(gps_coord);
        mGPSPosesCloud.points.push_back(
            PointT(gps_translated.x, gps_translated.y, gps_translated.z));
    }
}

void PointCloudProjector::Run()
{
    ROS_INFO("Starting point_cloud_projector ... \n");

    mPredictPoseSub = mNodeHandle.subscribe(
        "/predict_pose", 1000, &PointCloudProjector::PredictPoseCallback, this);
    mNmeaSub = mNodeHandle.subscribe(
        "/nmea_sentence", 1000, &PointCloudProjector::NMEACallback, this);
    mFixSub = mNodeHandle.subscribe(
        "/fix_vs330", 1000, &PointCloudProjector::FixCallback, this);

    ros::spin();

    // post-processing after killing point_cloud_projector node
    ROS_INFO("point_cloud_projector shutting down, writing transformations ...\n");
    ROS_INFO("projecting predict poses (size: %ld) to gps poses (size: %ld)\n",
        mPredictPosesCloud.points.size(), mGPSPosesCloud.points.size());

    size_t num_predict_poses (mPredictPosesCloud.points.size()),
           num_gps_poses (mGPSPosesCloud.points.size());
    if (num_gps_poses > num_predict_poses)
        mGPSPosesCloud.points.resize(num_predict_poses);
    else if (num_predict_poses > num_gps_poses)
        mPredictPosesCloud.points.resize(num_gps_poses);

    if (num_predict_poses > 0 and num_gps_poses > 0)
        mTransEstimator.estimateRigidTransformation(
            mPredictPosesCloud, mGPSPosesCloud, mRigidTransformation);

    std::cout << "Transformation Matrix: " << std::endl
              << mRigidTransformation << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr (
        new pcl::PointCloud<pcl::PointXYZI>);
    ROS_INFO("Transforming map file: %s\n", mMapFilename.c_str());
    pcl::io::loadPCDFile<pcl::PointXYZI> (mMapFilename, *map_cloud_ptr);
    pcl::transformPointCloud (*map_cloud_ptr, *map_cloud_ptr, mRigidTransformation);
    std::string projected_map_filename (mMapFilename + "_projected");
    ROS_INFO("Writing map to %s\n", projected_map_filename.c_str());
    pcl::io::savePCDFileBinary (projected_map_filename, *map_cloud_ptr);
}
