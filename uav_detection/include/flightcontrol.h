#ifndef __FLIGHTCONTROL_H
#define __FLIGHTCONTROL_H

#include <ros/ros.h>
#include <dji_sdk/dji_drone.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include "coordinatetransform.h"


namespace flight_control{

enum FlightState{
    S_NONE = 0,
    S_PREPARE, // Set gimbal
    S_TAKEOFF, // Take off
    S_SCAN, // Scan and detect target
    S_LAND, // Land
    S_ENDING // Wait for ending
};

class FlightControl
{
public:
    // Interface function
    bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
    void spin();

private:
    // Call back
    void RcChannelsCallback(const dji_sdk::RCChannels& msg);
    void GlobalPositionCallback(const dji_sdk::GlobalPosition& msg);
    void LocalPositionCallback(const dji_sdk::LocalPosition& msg);
    void AttitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion& msg);

    void PublishFlightState(int8_t state);
    void CalcWaypoint();
    void ScanWaypoint();

private:
    // Object
    DJIDrone* drone;
    ros::Subscriber rcchannels_sub;
    ros::Subscriber globalposition_sub;
    ros::Subscriber localposition_sub;
    ros::Subscriber attitudequaternion_sub;
    ros::Publisher state_pub;

    ct::GlobalPosition refglobalpos;
    ct::LocalPosition initlocalpos;
    ct::Attitude initattitude;
    std::vector<ct::LocalPosition> localwaypoints;
    std::vector<ct::GlobalPosition> globalwaypoints;

    // Parameters
    bool isStart;
    bool isGetrefglobalpos;
    bool isGetinitlocalpos;
    bool isGetinitattitude;

    std::vector<double> xlim; // Scan x limit
    std::vector<double> ylim; // Scan y limit
    double height; // Scan height
    double xstep; // Scan x step
    int scanmode; // Scan mode
    bool isRecordvideo;
    double velocity_range;
    double idle_velocity;
};

}

#endif // __FLIGHTCONTROL_H
