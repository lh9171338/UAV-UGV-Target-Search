#include "sendtarget.h"


namespace send_target{

bool SendTarget::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    // Parameters
    nh_priv.param<string>("serial_port", serialconfig.port, "/dev/ttyUSB0");
    nh_priv.param<int>("serial_baudrate", serialconfig.baudrate, 115200);

    // Start serial
    if(!serialtransmission.StartSerial(serialconfig))
    {
        return false;
    }

    // Initial target position for test
    double targets[5][2] = {
            {30.540149, 114.352361},
            {30.540139, 114.352371},
            {30.540129, 114.352361},
            {30.540139, 114.352351},
            {30.540139, 114.352361}
    };
    ct::GlobalPosition globalpos;
    for(int i = 0;i < 5;i++)
    {
        globalpos.latitude = targets[i][0];
        globalpos.longitude = targets[i][1];
        globalposList.push_back(globalpos);
	ROS_INFO("Target %d global position: %f, %f", i , globalpos.latitude, globalpos.longitude);
    }

    // Send target data
    if(SendTargetData())
    {
        ROS_INFO("Send target position successfully");
    }
    else
    {
        ROS_ERROR("Failed to send target position");
    }

    return true;
}

bool SendTarget::SendTargetData()
{
    vector<double> data;
    ct::GlobalPosition globalpos;

    for(int i = 0;i < globalposList.size();i++)
    {
        globalpos = globalposList[i];
        data.push_back(globalpos.latitude);
        data.push_back(globalpos.longitude);
    }

    return serialtransmission.SendDataWithSize(data);
}

};




