cmake_minimum_required(VERSION 2.8)
project(pointcloud_processing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

find_package(PCL 1.8 REQUIRED COMPONENTS common io)
message("Pcl found in ${PCL_LIBRARY_DIRS}")
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pointcloud_publisher publisher.cpp)
add_executable(pointcloud_subscriber subscriber.cpp)
add_executable(octomap_processing octomap_processing.cpp)
target_link_libraries(pointcloud_publisher ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(pointcloud_subscriber ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(octomap_processing ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES})
