#include <algorithm>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <experimental/filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::experimental::filesystem;

int main (int argc, char** argv)
{
    Json::Value arrayObj;
    int numSubmaps = 0;
    int totalPoints = 0;
    for (auto & inputFile : fs::directory_iterator(argv[1]))
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
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "    ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    std::ofstream outputFileStream("out/submap_config.json");
    writer -> write(rootJsonValue, &outputFileStream);

    return (0);
}
