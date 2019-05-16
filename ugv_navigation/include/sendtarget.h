#ifndef SEND_TARGET_H
#define SEND_TARGET_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include "serialtransmission.h"
#include "coordinatetransform.h"


using namespace std;


namespace send_target{

class SendTarget
{
public:
    // Interface function
    bool setup(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
    // Send target position data
    bool SendTargetData();

private:
    // Objects
    serial_transmission::SerialConfig serialconfig;
    serial_transmission::SerialTransmission serialtransmission;

    vector<ct::GlobalPosition> globalposList;
};

}

#endif // SEND_TARGET_H
