cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

project(get_pcd_info)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

catkin_package()

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}  
)

add_definitions(${PCL_DEFINITIONS})

add_executable (${PROJECT_NAME} src/main.cpp)
target_link_libraries (${PROJECT_NAME} ${JSONCPP_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} -lstdc++fs)
