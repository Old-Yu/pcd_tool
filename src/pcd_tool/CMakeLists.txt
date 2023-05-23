cmake_minimum_required(VERSION 3.0.2)
project(pcd_tool)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(pcd_tool src/pcd_tool.cpp)
target_link_libraries(pcd_tool ${catkin_LIBRARIES})


