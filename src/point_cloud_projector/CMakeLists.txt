cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_projector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES point_cloud_projector
    CATKIN_DEPENDS geometry_msgs nmea_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
    DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_compile_options(
	${PROJECT_NAME} PUBLIC "-std=c++11;-Wall;-Werror"
)
