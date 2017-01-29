#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double wobbler_angle;
ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;
double scanning_upwards;
double last_wobbler_angle;
bool been_awhile;
double scan_callback_count;
double min_scan_callbacks;

double min_ang;
double max_ang;

// Gets the angle of the current motor position
void hokuyoMotorCallback(const std_msgs::Int16& message_holder)
{
    if (message_holder.data>4096)
    {
        ROS_WARN("Someone thinks that the wobbler motor is too noisy right now.");
    }
    else
    {
        // Load recieved data into a program variable
        wobbler_angle = message_holder.data;

        // Get the most updated value of the wobbler's min/max wobbling angle. The angle can change in the middle of operation.
        if(!nh_ptr->getParam("min_angle", min_ang))
        {
            ROS_WARN("Wobbler point cloud could not find wobbler minimum angle (min_ang) on param server. Using arbitrary value.");
            min_ang = 500;
        }

        if(!nh_ptr->getParam("max_angle", max_ang))
        {
            ROS_WARN("Wobbler point cloud could not find wobbler maximum angle (max_ang) on param server. Using arbitrary value.");
            max_ang = 1000;
        }


        scan_callback_count++;
        if(scan_callback_count > min_scan_callbacks)
        {
            been_awhile = true;
        }

        // Check if we have reached a maxima or minima in the motion of the wobbler.
        //    If we have, it means that we have completed a full sweep in one direction and are ready to send out the point cloud.
        if (wobbler_angle < last_wobbler_angle && scanning_upwards == true && been_awhile == true)// Studies show that the wobbler doesn't always actually reach the min/max values
        {
            scanning_upwards = false;
            ROS_INFO("Wobbler completed an upward sweep!");
            cloud.header.frame_id = "lidar_link";

            pubCloud_ptr->publish(cloud);
            cloud.clear(); // Reset the point cloud, prepping for another sweep.
            been_awhile = false;
        }
        else if (wobbler_angle > last_wobbler_angle && scanning_upwards == false && been_awhile == true) 
        {
            scanning_upwards = true;
            ROS_INFO("Wobbler completed a downward sweep!");
            cloud.header.frame_id = "lidar_link";

            pubCloud_ptr->publish(cloud);
            cloud.clear(); // Reset the point cloud, prepping for another sweep.
            been_awhile = false;
        }
        // WE get an initial up/down jitter that happens. Can be fixed here, but I dont wanna do that now.
        last_wobbler_angle = wobbler_angle;
        // If it was neither of these, then the sweep isn't completed yet and we shouldnt clear/publish
    }
}

void cloudCallback(const PointCloud::ConstPtr& cloud_holder)
{
    int cloud_size = cloud_holder->height * cloud_holder->width;

    for (int i = 0; i < cloud_size; i++)
    {
        // CORRECTS FOR SOME WEIRD REFLECTION ISSUE WE GET IN THE Y-AXIS. I KNOW ITS HACKY - TZ - 11/21/2016
        pcl::PointXYZ single_point(cloud_holder->points[i].x, -cloud_holder->points[i].y, cloud_holder->points[i].z);

        cloud.points.push_back(single_point);
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc,argv,"hokuyo_pcl_stitcher");
    // Init last wobbler angle to special value
    last_wobbler_angle = -9999999;
    ros::NodeHandle nh;
    nh_ptr = &nh;

    // Internal scanning/sweeping parameters
    bool scanning_upwards = true;
    bool been_awhile = false;
    min_scan_callbacks = (max_ang - min_ang) / 2; // Require that the minimum number of scans be half the total expected number of scans. ie the wobbler must move halfway through single sweep before sending new point cloud.
    scan_callback_count = 0;

    if(!nh_ptr->getParam("min_ang", min_ang))
    {
        ROS_WARN("Wobbler point cloud could not find wobbler minimum angle (min_ang) on param server. Using arbitrary value.");
        min_ang = 900;
    }

    if(!nh_ptr->getParam("max_ang", max_ang))
    {
        ROS_WARN("Wobbler point cloud could not find wobbler maximum angle (max_ang) on param server. Using arbitrary value.");
        max_ang = 1100;
    }

    ros::Subscriber my_subscriber_object = nh.subscribe("/dynamixel_motor1_ang", 1, hokuyoMotorCallback);
    ros::Subscriber my_subscriber_object2 = nh.subscribe("/wobbler_scan_cloud", 1, cloudCallback);

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/wobbler_3d_cloud", 1);
    pubCloud_ptr = &pubCloud;

    ros::spin();
    return 0;
}