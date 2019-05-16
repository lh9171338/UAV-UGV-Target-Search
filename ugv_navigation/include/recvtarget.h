#ifndef __RECVTARGET_H
#define __RECVTARGET_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "serialtransmission.h"
#include "coordinatetransform.h"


using namespace std;


namespace recv_target{

class RecvTarget
{
public:
    // Interface function
    bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
    void GPSCallback(const sensor_msgs::NavSatFix& msg);
    void IMUCallback(const sensor_msgs::Imu& msg);

    // Receive target position data
    bool RecvTargetData();
    // Process target position data
    void ProcessData();
    // Set client goal
    void SetGoal(ct::LocalPosition target, move_base_msgs::MoveBaseGoal& goal);

private:
    // Objects
    serial_transmission::SerialConfig serialconfig;
    serial_transmission::SerialTransmission serialtransmission;

    ros::Subscriber gps_sub; // subscribe GPS data
    ros::Subscriber imu_sub; // subscribe IMU data

    ct::GlobalPosition initglobalpos;
    double inityaw;
    vector<ct::GlobalPosition> globalposList;
    vector<ct::LocalPosition> localposList;

    // Parameters
    bool isGetinitglobalpos;
    bool isGetinityaw;
    string fix_frame;
    int executetime;
    int sleeptime;
};

}

#endif // __RECVTARGET_H
