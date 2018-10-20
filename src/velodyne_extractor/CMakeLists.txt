cmake_minimum_required(VERSION 2.8.3)
project(velodyne_extractor)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  sensor_msgs
  tf
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES velodyne_extractor
	CATKIN_DEPENDS pcl_conversions pcl_ros rosbag roscpp sensor_msgs tf
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
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_compile_options(
	${PROJECT_NAME} PUBLIC "-std=c++11;-Wall;-Werror"
)
