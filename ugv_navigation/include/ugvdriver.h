#ifndef UGVDRIVER_H
#define UGVDRIVER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include "serialtransmission.h"
#include "coordinatetransform.h"


using namespace std;

namespace ugv_driver{

class UgvDriver
{
public:
  // Interface function
  bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
  void spin();

private:
  // Velocity command callback.
  void VelCallback(const geometry_msgs::Twist& msg);

  void CalcOdometry();
  // Publish sensor data
  void PublishSensorData();
  // Receive sensor data
  bool RecvSensorData();
  // Send velocity data
  bool SendVelData();

private:
  // Objects
  serial_transmission::SerialConfig serialconfig;
  serial_transmission::SerialTransmission serialtransmission;

  ros::Subscriber vel_sub; // subscribe velocity data
  ros::Publisher imu_pub; // publish IMU data
  ros::Publisher odom_pub; // publish odometry data
  ros::Publisher gps_pub; // publish GPS data
  tf::TransformBroadcaster odom_broadcaster; // broadcast coordinate transform between odom frame and base_link frame

  geometry_msgs::Pose2D pose;
  nav_msgs::Odometry wheel_odom; // wheel odometry data
  nav_msgs::Odometry last_wheel_odom; // wheel odometry data
  geometry_msgs::Twist vel_twist;  // velocity data
  sensor_msgs::Imu imu;
  sensor_msgs::MagneticField magnetometer;
  ct::GlobalPosition globalpos;

  // Parameters
  double roll, pitch, yaw;
  bool isfirst;
  ros::Time current_time;
  ros::Time last_time;

  string base_frame;
  string odom_frame;
  bool pub_base_odom_transform;
  int odom_rate; // rate of publishing imu data
  int imu_accel_range; // IMU acceleration range
  int imu_gyro_range; // IMU gyroscope range
};

}

#endif // UGVDRIVER_H
