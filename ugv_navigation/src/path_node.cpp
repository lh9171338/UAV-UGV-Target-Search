//
// Created by lihao on 19-3-29.
//

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;


// Global variable
ros::Publisher path_pub;
nav_msgs::Path path;
string fixed_frame;
string odom_frame;
bool use_odom;


void OdomCallback(const nav_msgs::Odometry& msg)
{
    //ROS_INFO("Odom callback");

    // Publish path
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg.header;

    pose_stamped.pose = msg.pose.pose;
    path.poses.push_back(pose_stamped);

    path_pub.publish(path);
}

void PoseCallback(const geometry_msgs::PoseStamped& msg)
{
    //ROS_INFO("Pose callback");

    // Publish path
    path.poses.push_back(msg);

    path_pub.publish(path);
}


main (int argc, char **argv)
{
    ros::init (argc, argv, "path_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Get parameters
    nh_priv.param<string>("fixed_frame", fixed_frame, "map");
    nh_priv.param<bool>("use_odom", use_odom, true);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = fixed_frame;

    // Subscribe odometer message
    ros::Subscriber odom_sub;
    ros::Subscriber pose_sub;
    if(use_odom)
    {
        odom_sub = nh.subscribe("odom", 10, &OdomCallback);
    }
    else
    {
        pose_sub = nh.subscribe("pose", 10, &PoseCallback);
    }

    // Publish path message
    path_pub = nh.advertise<nav_msgs::Path>("trajectory", 10);

    // Loop and execute callback function
    ros::spin();

    return 0;
}
