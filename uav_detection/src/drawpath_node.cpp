//
// Created by lihao on 19-3-29.
//

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;


// Global variable
ros::Publisher path_pub;
nav_msgs::Path path;
geometry_msgs::PoseStamped pose_stamped;
string fixed_frame;
string odom_frame;


void LocalPositionCallback(const dji_sdk::LocalPosition& msg)
{
    //ROS_INFO("LocalPositionCallback callback");

    // Publish path
    pose_stamped.header = msg.header;
    pose_stamped.pose.position.x = msg.x;
    pose_stamped.pose.position.y = -msg.y;
    pose_stamped.pose.position.z = msg.z;
    path.poses.push_back(pose_stamped);

    path_pub.publish(path);
}

void AttitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion& msg)
{
    //ROS_INFO("AttitudeQuaternionCallback callback");

    pose_stamped.pose.orientation.x = msg.q1;
    pose_stamped.pose.orientation.y = msg.q2;
    pose_stamped.pose.orientation.z = msg.q3;
    pose_stamped.pose.orientation.w = msg.q0;
}


main (int argc, char **argv)
{
    ros::init (argc, argv, "drawpath_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Get parameters
    nh_priv.param<string>("fixed_frame", fixed_frame, "map");

    path.header.stamp = ros::Time::now();
    path.header.frame_id = fixed_frame;

    // Subscribe odometer message
    ros::Subscriber localposition_sub;
    ros::Subscriber attitudequaternion_sub;
    localposition_sub = nh.subscribe("dji_sdk/local_position", 10, &LocalPositionCallback);
    attitudequaternion_sub = nh.subscribe("dji_sdk/attitude_quaternion", 10, &AttitudeQuaternionCallback);

    // Publish path message
    path_pub = nh.advertise<nav_msgs::Path>("trajectory", 10);

    // Loop and execute callback function
    ros::spin();

    return 0;
}
