cmake_minimum_required(VERSION 3.0.2)
project(lidar_dynamic_adjustment)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
ADD_DEFINITIONS(-DBoost_USE_STATIC_LIBS=ON)

set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
find_package(Boost REQUIRED)

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  rospy
  pcl_ros
  pcl_conversions
  
)

find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIRS})
#link_directories(${EIGEN3_LIBRARY_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})





generate_dynamic_reconfigure_options(
	cfg/lidar_dym.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_dynamic_adjustment
  CATKIN_DEPENDS 
  roscpp 
  sensor_msgs 
  std_msgs 
  dynamic_reconfigure

  pcl_ros
  pcl_conversions
  
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  #${Boost_LIBRARIES}
  #${PCL_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lidar_dynamic_adjustment.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


add_executable(${PROJECT_NAME}_node src/lidar_dym.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARY_DIRS}
    )


