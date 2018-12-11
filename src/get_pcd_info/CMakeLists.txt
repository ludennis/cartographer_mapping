cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++17)

project(get_pcd_info)

find_package(PCL 1.2 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)
link_directories(
    ${PCL_LIBRARY_DIRS}  
)

add_definitions(${PCL_DEFINITIONS})

add_executable (get_pcd_info src/main.cpp)
target_link_libraries (get_pcd_info ${JSONCPP_LIBRARIES} ${PCL_LIBRARIES} -lstdc++fs)
