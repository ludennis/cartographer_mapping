cmake_minimum_required(VERSION 2.8.3)
project(create_submap)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS runtime_manager
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(create_submap src/create_submap.cpp)

target_link_libraries(create_submap ${catkin_LIBRARIES})


