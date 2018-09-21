cmake_minimum_required(VERSION 2.8.3)
project(gps_coverage_plotter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  rosbag
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gps_coverage_plotter
#  CATKIN_DEPENDS geometry_msgs nmea_msgs rosbag rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS bin/gps_coverage_plotter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
