#include<ros/ros.h>

#include<pcl/io/pcd_io.h>
#include<pcl/point_cloud.h>
#include<pcl/filters/passthrough.h>  
#include<pcl/filters/voxel_grid.h> 
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/filters/statistical_outlier_removal.h>

#include<sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZI PointType;   

ros::Publisher pcd_tool_pub;
ros::Publisher pointCloud_pub;

std::string input_pcd_path;
std::string output_frame_id;
std::string output_pointcloud_topic;
std::string output_pcd_path;

bool crop_box_filter_enable,voxel_grid_filter_enable,remove_outliers_enable,rotate_translate_enable;
double crop_box_filter_min_x, crop_box_filter_max_x, crop_box_filter_min_y, crop_box_filter_max_y, crop_box_filter_min_z, crop_box_filter_max_z;  
double voxel_size_x,voxel_size_y,voxel_size_z;
int sor_nearby_number;
double sor_thresh_value;
double rotate_value_x, rotate_value_y, rotate_value_z;
double translate_value_x, translate_value_y, translate_value_z;


//角度制转弧度制
double rad(double d)
{
	return d * 3.1415926 / 180.0;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"pcd_tool");
    ros::NodeHandle nh;

    nh.param<bool>("crop_box_filter_enable",crop_box_filter_enable,false);
    nh.param<bool>("voxel_grid_filter_enable",voxel_grid_filter_enable,false);
    nh.param<bool>("remove_outliers_enable",remove_outliers_enable,false);
    nh.param<bool>("rotate_translate_enable",rotate_translate_enable,true);

    nh.param<double>("crop_box_filter_min_x",crop_box_filter_min_x,-10.0);
    nh.param<double>("crop_box_filter_max_x",crop_box_filter_max_x,10.0);
    nh.param<double>("crop_box_filter_min_y",crop_box_filter_min_y,-10.0);
    nh.param<double>("crop_box_filter_max_y",crop_box_filter_max_y,10.0);
    nh.param<double>("crop_box_filter_min_z",crop_box_filter_min_z,-10.0);
    nh.param<double>("crop_box_filter_max_z",crop_box_filter_max_z,10.0);

    nh.param<double>("voxel_size_x",voxel_size_x,0.1);
    nh.param<double>("voxel_size_y",voxel_size_y,0.1);
    nh.param<double>("voxel_size_z",voxel_size_z,0.1);

    nh.param<int>("sor_nearby_number",sor_nearby_number,30);
    nh.param<double>("sor_thresh_value",sor_thresh_value,1.0);

    nh.param<double>("rotate_value_x",rotate_value_x,0.0);
    nh.param<double>("rotate_value_y",rotate_value_y,0.0);
    nh.param<double>("rotate_value_z",rotate_value_z,0.0);

    nh.param<double>("translate_value_x",translate_value_x,0.0);
    nh.param<double>("translate_value_y",translate_value_y,0.0);
    nh.param<double>("translate_value_z",translate_value_z,0.0);

    nh.param<std::string>("output_frame_id", output_frame_id, "map");
    nh.param<std::string>("input_pcd_path", input_pcd_path, "./pcd_tool/pcd/test.pcd");
    nh.param<std::string>("output_pointcloud_topic", output_pointcloud_topic, "/output_point");
    nh.param<std::string>("output_pcd_path", output_pcd_path, "./pcd_tool/pcd/output.pcd"); 

    pcd_tool_pub = nh.advertise<sensor_msgs::PointCloud2> ("/input_cloud",10);          
    pointCloud_pub = nh.advertise<sensor_msgs::PointCloud2> (output_pointcloud_topic,10); 

    pcl::PointCloud<PointType>::Ptr pcd_cloud_in (new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (input_pcd_path, *pcd_cloud_in) == -1)
    {
    PCL_ERROR ("Couldn't read file: %s \n", input_pcd_path.c_str());
    return (-1);
    }
    sensor_msgs::PointCloud2 input_cloud;
    pcl::toROSMsg(*pcd_cloud_in,input_cloud);
    input_cloud.header.frame_id = output_frame_id;

    pcl::PointCloud<PointType>::Ptr handle_cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filter_z (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr vox_cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr filtered_cloud (new pcl::PointCloud<PointType>);

    if(crop_box_filter_enable)
    {
        //  直通滤波器 X 轴滤波 
        pcl::PointCloud<PointType>::Ptr filter_x (new pcl::PointCloud<PointType>);
        pcl::PassThrough<PointType> ptx;
        ptx.setInputCloud(pcd_cloud_in);                 //输入点云
        ptx.setFilterFieldName("x");                     //对x轴进行操作
        ptx.setFilterLimits(crop_box_filter_min_x, crop_box_filter_max_x);     //设置直通滤波器操作范围
        // ptx.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
        ptx.filter(*filter_x);                           //执行滤波，过滤结果保存在filter_x

        //  直通滤波器 Y 轴滤波 
        pcl::PointCloud<PointType>::Ptr filter_y (new pcl::PointCloud<PointType>);
        pcl::PassThrough<PointType> pty;
        pty.setInputCloud(filter_x);                     //输入点云
        pty.setFilterFieldName("y");                     //对y轴进行操作
        pty.setFilterLimits(crop_box_filter_min_y, crop_box_filter_max_y);     //设置直通滤波器操作范围
        // pty.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
        pty.filter(*filter_y);                           //执行滤波，过滤结果保存在filter_y

        //  直通滤波器 Z 轴滤波 
        pcl::PassThrough<PointType> ptz;
        ptz.setInputCloud(filter_y);                     //输入点云
        ptz.setFilterFieldName("z");                     //对z轴进行操作
        ptz.setFilterLimits(crop_box_filter_min_z, crop_box_filter_max_z);     //设置直通滤波器操作范围
        // ptz.setFilterLimitsNegative(true);              //设置保留范围内，还是过滤掉范围内
        ptz.filter(*filter_z);                           //执行滤波，过滤结果保存在filter_z
        filter_x.reset(new pcl::PointCloud<PointType>());
        filter_y.reset(new pcl::PointCloud<PointType>());
    }
    else{
        filter_z = pcd_cloud_in;
    }

    if(voxel_grid_filter_enable)
    {
         //  体素降采样
        pcl::VoxelGrid<PointType> vox_grid;
        vox_grid.setInputCloud(filter_z);
        vox_grid.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z); //设置滤波时创建的体素立方体(m)
        vox_grid.filter(*vox_cloud);
    }
    else{
        vox_cloud = filter_z;
    }
           
    if(remove_outliers_enable)
    {
        //  statisticalOutlierRemoval滤波器移除离群点
        pcl::StatisticalOutlierRemoval<PointType> sor;   
        sor.setInputCloud(vox_cloud);                                                 
        sor.setMeanK(sor_nearby_number);                            //设置在进行统计时考虑查询点临近点数                                              
        sor.setStddevMulThresh(sor_thresh_value);                   //设置判断是否为离群点的阀值                                           
        sor.filter(*filtered_cloud);  
    }
    else{
        filtered_cloud = vox_cloud;
    }

    if(rotate_translate_enable)
    {
        for(int i = 0; i < filtered_cloud->points.size(); i++) 
        {
            PointType new_point;
            double rx_x,rx_y,rx_z,ry_x,ry_y,ry_z,rz_x,rz_y,rz_z;
            double px = filtered_cloud->points[i].x;
            double py = filtered_cloud->points[i].y;
            double pz = filtered_cloud->points[i].z;
            double pi = filtered_cloud->points[i].intensity;
            //  点云绕 X 旋转
            rx_x = px;
            rx_y = cos(rad(rotate_value_x))*py + (-sin(rad(rotate_value_x)))*pz;
            rx_z = sin(rad(rotate_value_x))*py + cos(rad(rotate_value_x))*pz;

            //  点云绕 Y 旋转
            ry_x = cos(rad(rotate_value_y))*rx_x + (-sin(rad(rotate_value_y)))*rx_z;
            ry_y = rx_y;
            ry_z = sin(rad(rotate_value_y))*rx_x + cos(rad(rotate_value_y))*rx_z;

            //  点云绕 Z 旋转
            rz_x = cos(rad(rotate_value_z))*ry_x + (-sin(rad(rotate_value_z)))*ry_y;
            rz_y = sin(rad(rotate_value_z))*ry_x + cos(rad(rotate_value_z))*ry_y;
            rz_z = ry_z;

            //  点云整体平移
            new_point.x = rz_x + translate_value_x;
            new_point.y = rz_y + translate_value_y;
            new_point.z = rz_z + translate_value_z;
            new_point.intensity = pi;
            handle_cloud->points.push_back(new_point);
        }

    }
    else{
        for(int i = 0; i < filtered_cloud->points.size(); i++) 
        {
            PointType new_point;
            new_point.x = filtered_cloud->points[i].x;
            new_point.y = filtered_cloud->points[i].y;
            new_point.z = filtered_cloud->points[i].z;
            new_point.intensity = filtered_cloud->points[i].intensity;
            handle_cloud->points.push_back(new_point);
        }
    }
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*handle_cloud,output_cloud);
    output_cloud.header.frame_id = output_frame_id;
    pcl::io::savePCDFileBinary(output_pcd_path, *handle_cloud);     // 保存为新的PCD点云

    handle_cloud.reset(new pcl::PointCloud<PointType>());
    filter_z.reset(new pcl::PointCloud<PointType>());
    vox_cloud.reset(new pcl::PointCloud<PointType>());
    filtered_cloud.reset(new pcl::PointCloud<PointType>());

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcd_tool_pub.publish(input_cloud);
        pointCloud_pub.publish(output_cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

