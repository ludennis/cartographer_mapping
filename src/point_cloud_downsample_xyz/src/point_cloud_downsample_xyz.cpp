#include <iostream>
#include <fstream>

#include <vector>
#include <string>

#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

using namespace std;

namespace boost_po = boost::program_options;

std::vector<std::string> map_filenames;
double opacity;

int main(int argc, char** argv)
{
    boost_po::options_description descriptions{"Allowed options"};
    descriptions.add_options()
        ("help", "produce help message")
        ("opacity", boost_po::value<double>()->default_value(1.0),
            "set opacity(alpha) value")
        ("map-filenames", boost_po::value<std::vector<std::string>>()->multitoken(),
            "name(s)");

    boost_po::variables_map var_map;
    boost_po::store(boost_po::parse_command_line(argc, argv, descriptions), var_map);
    boost_po::notify(var_map);

    if(var_map.count("help"))
    {
        ROS_INFO_STREAM(std::endl << descriptions);
        return 1;
    }

    if (var_map.count("opacity"))
    {
        ROS_INFO_STREAM("Opacity set to: " << var_map["opacity"].as<double>());
        opacity = var_map["opacity"].as<double>();
    }

    if (var_map.count("map-filenames"))
    {
        map_filenames = var_map["map-filenames"].as<std::vector<std::string>>();

        for(auto itr = map_filenames.begin(); itr != map_filenames.end(); ++itr)
        {
            ROS_INFO_STREAM("Map filenames: " << *itr);
        }
        ROS_INFO_STREAM("Load Map Number : " << map_filenames.size());
    }
    else
    {
        ROS_ERROR("No map files given, exiting");
        return -1;
    }

    for (auto itr = map_filenames.begin(); itr != map_filenames.end(); ++itr)
    {
        ifstream input_file( *itr );
        std::string input_filename = *itr;
        std::string output_filename = input_filename.erase(input_filename.length()-4);
        output_filename.append("_ds.xyz");
        ofstream output_file(output_filename);

        int count=0;
        if(input_file && output_file)
        {
            ROS_INFO_STREAM("down sample " << input_filename );
            string line;
            while(getline(input_file,line))
            {
               if(count % 100 == 0)
               {
                  output_file << line << endl;
                  //ROS_INFO_STREAM("count=" << count << " -  " << line);
               }
               count++;
            }
            input_file.close();
            output_file.close();
        }
    }

    return 0;
}
