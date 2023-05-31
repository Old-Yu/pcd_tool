#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
 
#include <pcl_ros/transforms.h>
#include <dynamic_reconfigure/server.h>
#include <lidar_dynamic_adjustment/lidar_dymConfig.h>  




typedef pcl::PointXYZI pointT;

ros::Publisher lidar_pub;
std::string base_frame_id = "map";
struct lidar_param_{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
}lidar_param;

sensor_msgs::PointCloud2::ConstPtr lidar_transformed_pc(new sensor_msgs::PointCloud2); 

void callback(lidar_dynamic_adjustment::lidar_dymConfig &config, uint32_t level) 
{
    lidar_param.x = config.x;
    lidar_param.y = config.y;
    lidar_param.z = config.z;
    lidar_param.roll = config.roll;
    lidar_param.pitch = config.pitch;
    lidar_param.yaw = config.yaw;
}



void lidar_base_callback(const sensor_msgs::PointCloud2::ConstPtr &lidar_base_msg){
                        
    //sensor_msgs::PointCloud2::ConstPtr lidar_base_msg_in;
    std::cout<<"111111"<<std::endl;
    pcl::PointCloud<pointT>::Ptr cloud_final(new pcl::PointCloud<pointT>);
    pcl::PointCloud<pointT>::Ptr base_points(new pcl::PointCloud<pointT>);
    pcl::PointCloud<pointT>::Ptr child_points(new pcl::PointCloud<pointT>);

    pcl::PCLPointCloud2::Ptr base_points_blob(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr child_points_blob(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_final_blob(new pcl::PCLPointCloud2);

    

    pcl::fromROSMsg(*lidar_base_msg, *base_points);
    pcl::fromROSMsg(*lidar_transformed_pc, *child_points);

    pcl::toPCLPointCloud2(*base_points, *base_points_blob);
    pcl::toPCLPointCloud2(*child_points, *child_points_blob);

    
    if(base_points->points.size()>0){
        std::cout<<"lidar_base point size is =="<<base_points->points.size()<<std::endl;
        if(child_points->points.size()>0){
            if(pcl::concatenatePointCloud(*base_points_blob, *child_points_blob, *cloud_final_blob)){
                sensor_msgs::PointCloud2::Ptr out_cloud_msg(new sensor_msgs::PointCloud2());
                pcl::PointCloud<pointT>::Ptr cloud_final(new pcl::PointCloud<pointT>);
                pcl::fromPCLPointCloud2(*cloud_final_blob, *cloud_final);
                pcl::toROSMsg(*cloud_final,*out_cloud_msg);
                out_cloud_msg->header.frame_id = base_frame_id;
                out_cloud_msg->header.stamp = ros::Time::now();
                lidar_pub.publish(out_cloud_msg);
            }
            //pclPointCloud2 ->pclPointCloud
            


        }
        else{
            lidar_pub.publish(lidar_base_msg);
        }
        
       
        
    }else{
       return ;
    }
    // concat_cloud->header = lidar_base_msg->header;
    // lidar_pub.publish(std::move(concat_cloud));
   
}

void lidar_child_callback(const sensor_msgs::PointCloud2ConstPtr &lidar_child_msg){
    
       std::cout<<"2222"<<std::endl;
       sensor_msgs::PointCloud2::Ptr lidar_child_msg_out(new sensor_msgs::PointCloud2());
       Eigen::AngleAxisf rotation_x(lidar_param.roll, Eigen::Vector3f::UnitX());
       Eigen::AngleAxisf rotation_y(lidar_param.pitch, Eigen::Vector3f::UnitY());
       Eigen::AngleAxisf rotation_z(lidar_param.yaw, Eigen::Vector3f::UnitZ());
       Eigen::Translation3f translation(lidar_param.x, lidar_param.y, lidar_param.z);
       Eigen::Matrix4f rotation_matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();

       pcl_ros::transformPointCloud(rotation_matrix, *lidar_child_msg, *lidar_child_msg_out);
       lidar_transformed_pc = lidar_child_msg_out;
       std::cout<<"transform is finished!!"<<std::endl;
}



int main(int argc, char **argv) 
{
  ros::init(argc, argv, "lidar_dunamic_");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<lidar_dynamic_adjustment::lidar_dymConfig> server;
  dynamic_reconfigure::Server<lidar_dynamic_adjustment::lidar_dymConfig>::CallbackType f;
  
  std::string lidar_base_topic_name = "/input_cloud";
  std::string lidar_child_topic_name = "/output_point";

  
  ros::Subscriber lidar_base  = nh.subscribe<sensor_msgs::PointCloud2>(lidar_base_topic_name, 1, lidar_base_callback);
  ros::Subscriber lidar_child = nh.subscribe<sensor_msgs::PointCloud2>(lidar_child_topic_name,1, lidar_child_callback);
  lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/lidar_concat_output",100);

  ROS_INFO("Spinning node");
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();   
    loop_rate.sleep();
  }
  //ros::spin();
  return 0;
}
