cmake_minimum_required(VERSION 2.8.3)
project(laneline_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp
)

#########
# Build #
#########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

target_compile_options(
    ${PROJECT_NAME} PUBLIC "-std=c++11;-Wall;-Werror"
)
