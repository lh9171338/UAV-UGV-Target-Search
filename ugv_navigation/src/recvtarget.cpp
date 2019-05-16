#include "recvtarget.h"


namespace recv_target{

bool RecvTarget::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    isGetinitglobalpos = false;
    isGetinityaw = false;
    ros::Rate loop_rate(10);

    // Parameters
    nh_priv.param<string>("serial_port", serialconfig.port, "/dev/ttyUSB0");
    nh_priv.param<int>("serial_baudrate", serialconfig.baudrate, 115200);
    nh_priv.param<string>("fix_frame", fix_frame, "map");
    nh_priv.param<int>("executetime", executetime, 100);
    nh_priv.param<int>("sleeptime", sleeptime, 5);

    // Subscribe topics
    gps_sub = nh.subscribe("gps", 10, &RecvTarget::GPSCallback, this);
    imu_sub = nh.subscribe("imu", 10, &RecvTarget::IMUCallback, this);

    // Start serial
    if(!serialtransmission.StartSerial(serialconfig))
    {
        return false;
    }

    // Start clinet
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    client.waitForServer();

    // Obtain original global position and attitude
    while(!(isGetinitglobalpos && isGetinityaw))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Wait for target data
    ROS_INFO("Wait for target data...");
    if(!RecvTargetData())
    {
        ROS_ERROR("Failed to receive target position");
        return false;
    }

    // Process data
    ProcessData();

    // Send goal
    move_base_msgs::MoveBaseGoal goal;
    for(int i = 0;i < localposList.size();i++)
    {
        SetGoal(localposList[i], goal);
        client.sendGoal(goal);
        client.waitForResult(ros::Duration(executetime));
        if(client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_ERROR("Failed to move to the target");
            return false;
        }
        sleep(sleeptime);
    }

    return true;
}

void RecvTarget::GPSCallback(const sensor_msgs::NavSatFix& msg)
{
    if(!isGetinitglobalpos && msg.latitude != 0 && msg.longitude != 0)
    {
        initglobalpos.latitude = msg.latitude;
        initglobalpos.longitude = msg.longitude;
        initglobalpos.altitude = msg.altitude;
        isGetinitglobalpos = true;

        ROS_INFO("Get initial global position successfully");
        ROS_INFO("Global position: %f, %f, %f", msg.latitude, msg.longitude, msg.altitude);
    }
}

void RecvTarget::IMUCallback(const sensor_msgs::Imu& msg)
{
    if(!isGetinityaw)
    {
        double roll, pitch, yaw;
        tf::Quaternion quat(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        inityaw = 90 - RAD2DEG(yaw);
        isGetinityaw = true;

        ROS_INFO("Get initial yaw successfully");
        ROS_INFO("yaw: %f", inityaw);
    }
}

bool RecvTarget::RecvTargetData()
{
    vector<double> data;

    if(!serialtransmission.RecvDataWithSize(data))
    {
        return false;
    }
    ct::GlobalPosition globalpos;
    for(int i = 0;i < data.size();i+=2)
    {
        globalpos.latitude = data[i];
        globalpos.longitude = data[i + 1];
        globalpos.altitude = 0;
        globalposList.push_back(globalpos);
        ROS_INFO("Target %d global position: %f, %f", i / 2 + 1, globalpos.latitude, globalpos.longitude);
    }

    return true;
}

void RecvTarget::ProcessData()
{
    ct::LocalPosition localpos;
    ct::LocalPosition rotatelocalpos;
    for(int i = 0;i < globalposList.size();i++)
    {
        ct::GlobalToLocal(initglobalpos, globalposList[i], localpos);
        ct::YawRotate(-inityaw, localpos, rotatelocalpos);
        rotatelocalpos.y = -rotatelocalpos.y;
        localposList.push_back(rotatelocalpos);
        ROS_INFO("Target %d local position: %f, %f", i / 2 + 1, rotatelocalpos.x, rotatelocalpos.y);
    }
}

void RecvTarget::SetGoal(
        ct::LocalPosition               target,
        move_base_msgs::MoveBaseGoal&	goal
)
{
    goal.target_pose.header.frame_id = fix_frame;
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = target.x;
    goal.target_pose.pose.position.y = target.y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    ROS_INFO("Set target: %f, %f", target.x, target.y);
}

};




