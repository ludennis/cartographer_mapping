#include <string>
#include <algorithm>
#include <iostream>
#include <regex>
#include <vector>
#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <boost/multi_array.hpp>
#include <jsoncpp/json/json.h>
#include <experimental/filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace boost_po = boost::program_options;
namespace fs = std::experimental::filesystem;

std::string mapDirectiry;
std::string strGpsReferenceArray;
std::vector<std::string> gpsReferenceArray;

int main (int argc, char** argv)
{
    boost_po::options_description description{"Allowed options"};
    description.add_options()
        ("help", "get help message")
        ("map-directory", boost_po::value<std::string>(), "map directory")
        ("gps-reference", boost_po::value<std::string>()->default_value("nan"),
            "Please enter 'latitude,longitude,altitude'");

    boost_po::variables_map varMap;
    boost_po::store(boost_po::parse_command_line(argc, argv, description), varMap);
    boost_po::notify(varMap);

    if(varMap.count("help"))
    {
        ROS_INFO_STREAM(std::endl << description);
        return 1;
    }
    if(varMap.count("map-directory"))
    {
        mapDirectiry = varMap["map-directory"].as<std::string>();
        ROS_INFO_STREAM("Loading map file directory: " << mapDirectiry);
    } else
    {
        ROS_ERROR("No map file directory given, exiting");
        return -1;
    }
    if(varMap.count("gps-reference"))
    {
        strGpsReferenceArray = varMap["gps-reference"].as<std::string>();
        if (strGpsReferenceArray != "nan")
        {
            std::regex comma(",");
            std::vector<std::string> tmpStr(std::sregex_token_iterator(
                strGpsReferenceArray.begin(), strGpsReferenceArray.end(),
                comma, -1), std::sregex_token_iterator());
            if (tmpStr.size() <=3)
            {
                ROS_INFO_STREAM("GPS Reference:(" << tmpStr[0] << ", " <<
                    tmpStr[1] << ", " << tmpStr[2] << ")");

                for (auto tmpElemnt:tmpStr)
                {
                    gpsReferenceArray.push_back(tmpElemnt);
                }
            }
            else
            {
                ROS_ERROR_STREAM("Please check gps reference input:" <<
                    strGpsReferenceArray);
                return -1;
            }
        }
        else
        {
            ROS_WARN("GPS Reference will be set nan, please enter argument.");
            for (int i = 0; i < 3; ++i)
            {
                gpsReferenceArray.push_back("nan");
            }
        }
    }

    Json::Value arrayObj;
    int numSubmaps = 0;
    int totalPoints = 0;
    for (auto & inputFile : fs::directory_iterator(mapDirectiry))
    {
        std::cout << "processing " << inputFile << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        Json::Value tmpValue;

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (fs::path(inputFile).string(), *cloud) == -1)
        {
            PCL_ERROR ("Couldn't read file %s\n", fs::path(inputFile).string().c_str());
        }
        else
        {
            std::cout << "Loaded "
                    << cloud->width * cloud->height
                    << " data points from test_pcd.pcd with the following fields: "
                    << std::endl;

            std::vector<float> ptX;
            std::vector<float> ptY;
            for (size_t i = 0; i < cloud->points.size(); ++i)
            {
                ptX.push_back(cloud->points[i].x);
                ptY.push_back(cloud->points[i].y);
            }

            const float centerX = 0.5f * (
                *std::max_element(ptX.begin(), ptX.end()) +
                *std::min_element(ptX.begin(), ptX.end()));

            const float centerY = 0.5f * (
                *std::max_element(ptY.begin(), ptY.end()) +
                *std::min_element(ptY.begin(), ptY.end()));

            tmpValue["num_points"] = Json::Value(
                static_cast<int>(cloud->points.size()));
            tmpValue["file_name"] = fs::path(inputFile).filename().string().c_str();
            tmpValue["center_x"] = Json::Value(centerX);
            tmpValue["center_y"] = Json::Value(centerY);
            arrayObj.append(tmpValue);
            numSubmaps ++;
            totalPoints += cloud->points.size();
        }
    }

    Json::Value rootJsonValue;
    rootJsonValue["num_submaps"] = numSubmaps;
    rootJsonValue["total_points"] = totalPoints;
    rootJsonValue["submaps"] = arrayObj;
    Json::Value gpsObj;
    gpsObj["latitude"] = gpsReferenceArray[0];
    gpsObj["longitude"] = gpsReferenceArray[1];
    gpsObj["altitude"] = gpsReferenceArray[2];
    rootJsonValue["gps_reference"] = gpsObj;
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "    ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::string outFilename = mapDirectiry + "/submaps_config.json";
    std::cout << "Writing to file: " << outFilename << std::endl;
    std::ofstream outputFileStream(outFilename);
    writer -> write(rootJsonValue, &outputFileStream);

    return (0);
}
