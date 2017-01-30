// Ground plane finder main source code file for ROS point cloud processing
// gnd_plane_finder.cpp
// Created by Trent Ziemer January 29, 2017

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

// ROS PCL support
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Filestream and C Std Lib tools
#include <iostream>
#include <fstream>
#include <string>

// Numerical constant definitions
#define PI 3.14159265

// To simplify code.
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Normal ros global object declarations
ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;

double min_ang;
double max_ang;

// Filestream variables
std::ofstream * outputFile_ptr;

void hokuyoMotorCallback(const std_msgs::Int16& message_holder)
{
}

std::vector<int> fit_plane_to_cloud(const PointCloud::ConstPtr& point_cloud)
{
    std::vector<int> plane_normal;
    
    int max_iterations = 50; // I think this might be a good starting point

    for(int i = 0; i < max_iterations; i++)
    {



    }

    return plane_normal;
}

void cloudCallback(const PointCloud::ConstPtr& cloud_holder)
{
    std::cout << "Starting cloud processing callback on received point cloud." << std::endl;
    int cloud_size = cloud_holder->height * cloud_holder->width;

    for(int i = 0; i < cloud_size; i++)
    {
        //cloud_holder->points[i].x
        //cloud_holder->points[i].y
        //cloud_holder->points[i].z
        // Output data to ~/output.csv data file for whatever purpose you may need
        *outputFile_ptr << cloud_holder->points[i].x << "," << cloud_holder->points[i].y << cloud_holder->points[i].z << std::endl;
        std::vector<int> plane_normal;
        plane_normal = fit_plane_to_cloud(cloud_holder);
        std::cout << "PLANE NORMAL FOUND?" << std::endl;
    }
    outputFile_ptr->close();
    std::cout << "Done with a single cloud processing." << std::endl;
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"hokuyo_pcl_stitcher");

    ros::NodeHandle nh;
    nh_ptr = &nh;

    // Open point cloud data file for output
    std::ofstream outputFile;
    outputFile_ptr = &outputFile;
    outputFile_ptr->open("/home/mordoc/output.csv");

    if(!nh_ptr->getParam("min_ang", min_ang))
    {
        ROS_WARN("!!!!");
        min_ang = 900;
    }

    if(!nh_ptr->getParam("max_ang", max_ang))
    {
        ROS_WARN("!!!!!!");
        max_ang = 1100;
    }

    ros::Subscriber my_subscriber_object = nh.subscribe("/dynamixel_motor1_ang", 1, hokuyoMotorCallback);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("/wobbler_3d_cloud", 1, cloudCallback);

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/unused_topic", 1);
    pubCloud_ptr = &pubCloud;

    ros::spin();
    return 0;
}