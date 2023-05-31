# pcd_tool

#### 介绍
ROS环境下采用PCL点云库对PCD格式三维点云进行加载、滤波、旋转和平移等处理

```
mkdir catkin_ws
cd catkin_ws
git clone https://github.com/shaoniandujianyiqingqiu/pcd_tool.git
catkin_make

# PCD_TOOL
source devel/setup.bash
roslaunch pcd_tool pcd_tool.launch

# LIDAR_DYNAMIC_ADJUSTMENT
source devel/setup.bash
rosrun lidar_dynamic_adjustment lidar_dynamic_adjustment_node 
rosrun rqt_reconfigure rqt_reconfigure
```
