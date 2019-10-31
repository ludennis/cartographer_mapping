/*
Create submap from large map
Chun-Te, Dennis, Yu-Syuan
*/

#include <algorithm>
#include <iostream>
#include <regex>
#include <string>
#include <sstream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <boost/program_options.hpp>
#include <boost/multi_array.hpp>

#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <experimental/filesystem>
#include <jsoncpp/json/json.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef PointCloudT::Ptr PointCloudTPtr;
typedef boost::multi_array<PointCloudT, 2> PointCloudArray2D;
namespace boost_po = boost::program_options;
namespace fs = std::experimental::filesystem;

std::string fileFormat;
int submapSize;
std::string mapFilename;
std::string strGpsReferenceArray;
std::vector<std::string> gpsReferenceArray;
std::string submapsDirectory;

void WriteToJsonFile(
  const int numSubmaps, const int totalPoints, const Json::Value& arrayObj)
{
  Json::Value rootJsonValue;
  rootJsonValue["numSubmaps"] = numSubmaps;
  rootJsonValue["totalPoints"] = totalPoints;
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
  std::string outFilename = submapsDirectory + "/submaps_config.json";
  std::cout << "Writing to file: " << outFilename << std::endl;
  std::ofstream outputFileStream(outFilename);
  writer->write(rootJsonValue, &outputFileStream);
}

void CreateSubmapsWithConfig(
  const std::string& fileFormat, const int submapSize, const std::string& mapFileName)
{
  PointCloudTPtr inputCloud (new PointCloudT);
  if (pcl::io::loadPCDFile(mapFilename, *inputCloud) == -1)
  {
    ROS_ERROR_STREAM("Couldn't read file: " << mapFilename);
    return;
  }

  PointT mapMinPoint, mapMaxPoint;
  pcl::getMinMax3D (*inputCloud, mapMinPoint, mapMaxPoint);
  ROS_INFO_STREAM("" << "Max x: " << mapMaxPoint.x);
  ROS_INFO_STREAM("" << "Max y: " << mapMaxPoint.y);
  ROS_INFO_STREAM("" << "Max z: " << mapMaxPoint.z);
  ROS_INFO_STREAM("" << "Min x: " << mapMinPoint.x);
  ROS_INFO_STREAM("" << "Min y: " << mapMinPoint.y);
  ROS_INFO_STREAM("" << "Min z: " << mapMinPoint.z);

  const int xGridSize = (int) ceil((mapMaxPoint.x - mapMinPoint.x) / submapSize);
  const int yGridSize = (int) ceil((mapMaxPoint.y - mapMinPoint.y) / submapSize);

  PointCloudArray2D submaps(boost::extents[xGridSize][yGridSize]);

  for (unsigned int index = 0; index < inputCloud->size(); ++index)
  {
    const int xGrid =
      (int) floor( (inputCloud->points[index].x - mapMinPoint.x) / submapSize);
    const int yGrid =
      (int) floor( (inputCloud->points[index].y - mapMinPoint.y) / submapSize);
    submaps[xGrid][yGrid].points.push_back(inputCloud->points[index]);
  }

  std::string removedText = ".pcd";
  std::size_t textIndex = mapFilename.find(removedText);
  if (textIndex != std::string::npos)
  {
    mapFilename.erase(textIndex, removedText.length());
  }

  std::string outFileDirectory = mapFilename + "_submaps/";
  boost::filesystem::create_directories(outFileDirectory);

  for (int x = 0; x < xGridSize; ++x)
  {
    for (int y = 0; y < yGridSize; ++y)
    {
      if (submaps[x][y].size() > 0)
      {
        submaps[x][y].width = (int) submaps[x][y].points.size();
        submaps[x][y].height = 1;
        ROS_INFO("submaps[%d][%d]: points.size() = %ld, width = %d, "
          "height = %d, is_dense = %d", x, y, submaps[x][y].points.size(),
          submaps[x][y].width, submaps[x][y].height, submaps[x][y].is_dense);
        std::string submapFilename =
          "submap_" + std::to_string(x) + "_" + std::to_string(y) + ".pcd";

        if (fileFormat.compare("ascii") == 0)
        {
          if (pcl::io::savePCDFileASCII(outFileDirectory + submapFilename, submaps[x][y]) == -1)
            ROS_ERROR_STREAM("Failed saving: " << submapFilename);
        }
        else
        {
          if (pcl::io::savePCDFileBinary(outFileDirectory + submapFilename, submaps[x][y]) == -1)
            ROS_ERROR_STREAM("Failed saving: " << submapFilename);
        }
      }
    }
  }

  // Get submaps_config.json
  Json::Value arrayObj;
  int numSubmaps = 0;
  int totalPoints = 0;

  for (auto & inputFile : fs::directory_iterator(outFileDirectory))
  {
    std::cout << "processing " << inputFile << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    Json::Value tmp_value;

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

      tmp_value["numPoints"] = Json::Value(
        static_cast<int>(cloud->points.size()));
      tmp_value["fileName"] = fs::path(inputFile).filename().string().c_str();
      tmp_value["centerX"] = Json::Value(centerX);
      tmp_value["centerY"] = Json::Value(centerY);
      arrayObj.append(tmp_value);
      numSubmaps ++;
      totalPoints += cloud->points.size();
    }
  }
  WriteToJsonFile(numSubmaps, totalPoints, arrayObj);
}

void GenerateConfig(const std::string& submapsDirectory)
{
  Json::Value arrayObj;
  int numSubmaps = 0;
  int totalPoints = 0;
  for (auto & inputFile : fs::directory_iterator(submapsDirectory))
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

      tmpValue["numPoints"] = Json::Value(
        static_cast<int>(cloud->points.size()));
      tmpValue["fileName"] = fs::path(inputFile).filename().string().c_str();
      tmpValue["centerX"] = Json::Value(centerX);
      tmpValue["centerY"] = Json::Value(centerY);
      arrayObj.append(tmpValue);
      numSubmaps++;
      totalPoints += cloud->points.size();
    }
  }
  WriteToJsonFile(numSubmaps, totalPoints, arrayObj);
}

int main (int argc, char** argv)
{
  boost_po::options_description description{"Allowed options"};
  description.add_options()
    ("help", "get help message")
    ("file-format", boost_po::value<std::string>()->default_value("binary"),
        "file format, either 'ascii' or 'binary'")
    ("submap-size", boost_po::value<int>()->default_value(50),
        "size of each submap")
    ("map-filename", boost_po::value<std::string>(), "map filename")
    ("generate-config", boost_po::value<std::string>(),
        "Only generate config within a specified directory of submaps")
    ("gps-reference", boost_po::value<std::string>()->default_value("nan"),
        "Please enter 'latitude,longitude,altitude'");

  boost_po::variables_map var_map;
  boost_po::store(boost_po::parse_command_line(argc, argv, description), var_map);
  boost_po::notify(var_map);

  if (var_map.count("help"))
  {
    ROS_INFO_STREAM(std::endl << description);
    return 1;
  }

  if (var_map.count("gps-reference"))
  {
    strGpsReferenceArray = var_map["gps-reference"].as<std::string>();
    if (strGpsReferenceArray != "nan")
    {
      std::regex comma(",");
      std::vector<std::string> tmp_str(std::sregex_token_iterator(
        strGpsReferenceArray.begin(), strGpsReferenceArray.end(),
        comma, -1), std::sregex_token_iterator());
      if (tmp_str.size() <=3)
      {
        ROS_INFO_STREAM("GPS Reference:(" << tmp_str[0] << ", " <<
          tmp_str[1] << ", " << tmp_str[2] << ")");

        for (auto tmp_elemnt:tmp_str)
        {
          gpsReferenceArray.push_back(tmp_elemnt);
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

  if (!var_map.count("map-filename") && !var_map.count("generate-config"))
  {
    ROS_ERROR("Error: Please specify either option '--map-filename' or "
      "'--generate-config'");
    return -1;
  }

  if (var_map.count("map-filename") && var_map.count("generate-config"))
  {
    ROS_ERROR("Error: only one of options '--map-filename' or '--generate-config' "
      "can be specified");
    return -1;
  }

  if (var_map.count("map-filename"))
  {
    if (var_map.count("file-format"))
    {
      fileFormat = var_map["file-format"].as<std::string>();
      ROS_INFO_STREAM("Writing submaps with file format: " << fileFormat);
    }

    if (var_map.count("submap-size"))
    {
      submapSize = var_map["submap-size"].as<int>();
      ROS_INFO_STREAM("Writing submaps with each submap size: " << submapSize);
    }

    mapFilename = var_map["map-filename"].as<std::string>();
    ROS_INFO_STREAM("Loading map file: " << mapFilename);

    CreateSubmapsWithConfig(fileFormat, submapSize, mapFilename);
  }
  else if (var_map.count("generate-config"))
  {
    submapsDirectory = var_map["generate-config"].as<std::string>();
    ROS_INFO_STREAM("Generating submaps config file at: " << submapsDirectory);

    GenerateConfig(submapsDirectory);
  }

  return 0;
}
