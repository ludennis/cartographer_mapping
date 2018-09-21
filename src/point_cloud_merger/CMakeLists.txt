cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_merger)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_merger
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cc)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
