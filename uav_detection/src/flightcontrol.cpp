#include "flightcontrol.h"


namespace flight_control{

bool FlightControl::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    isStart = false;
		isGetrefglobalpos = false;
    isGetinitlocalpos = false;
    isGetinitattitude = false;
    ros::Rate loop_rate(50);

    // Parameters
    nh_priv.getParam("xlim", xlim);
    nh_priv.getParam("ylim", ylim);
    nh_priv.param<double>("height", height, 3.0);
    nh_priv.param<double>("xstep", xstep, 2.0);
    nh_priv.param<int>("scanmode", scanmode, 0);
    nh_priv.param<bool>("isRecordvideo", isRecordvideo, false);
    nh_priv.param<double>("velocity_range", velocity_range, 3);
    nh_priv.param<double>("idle_velocity", idle_velocity, 1);

    // Subscribe some topics
    rcchannels_sub = nh.subscribe("dji_sdk/rc_channels", 10, &FlightControl::RcChannelsCallback, this);
    globalposition_sub = nh.subscribe("dji_sdk/global_position", 10, &FlightControl::GlobalPositionCallback, this);
    localposition_sub = nh.subscribe("dji_sdk/local_position", 10, &FlightControl::LocalPositionCallback, this);
    attitudequaternion_sub = nh.subscribe("dji_sdk/attitude_quaternion", 10, &FlightControl::AttitudeQuaternionCallback, this);

    // Create an flight state publisher
    state_pub = nh.advertise<std_msgs::Int8>("state", 10);

    // Instantiates drone and obtain authority
    drone = new DJIDrone(nh);
    ROS_INFO("Wait for start signal...");
    while(!isStart) // wait for start signal
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    if(!drone->request_sdk_permission_control())
    {
        ROS_ERROR("Failed to request sdk permission of control");
        return false;
    }

    // Obtain original global position and attitude
    while(!(isGetrefglobalpos && isGetinitlocalpos && isGetinitattitude))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    CalcWaypoint();

    ROS_INFO("Setup flightcontrol node successfully");
    return true;
}

void FlightControl::spin()
{
    // Set gimbal
    ROS_INFO("Set gimbal...");
    PublishFlightState(S_PREPARE);
    drone->gimbal_angle_control(0.0, -900.0, 0.0, 50, 1); // mode: 0 Incremental control, 0 Absolute control
    sleep(5);

    // Take off
    ROS_INFO("Take off...");
    PublishFlightState(S_TAKEOFF);
    drone->takeoff();
    sleep(5);

    // Scan
    ROS_INFO("Scan...");
    PublishFlightState(S_SCAN);
    if(isRecordvideo)
    {
        drone->start_video();
    }
    ScanWaypoint();
    if(isRecordvideo)
    {
        drone->stop_video();
    }

    // Land
    ROS_INFO("Land...");
    PublishFlightState(S_LAND);
    drone->landing();
    sleep(10);

    // Wait for ending
    ROS_INFO("Wait for ending...");
    PublishFlightState(S_ENDING);

    drone->release_sdk_permission_control();

    ros::spin();
}

void FlightControl::RcChannelsCallback(const dji_sdk::RCChannels& msg)
{
    isStart = msg.mode == 8000;
}

void FlightControl::GlobalPositionCallback(const dji_sdk::GlobalPosition& msg)
{
    if(!isGetrefglobalpos)
    {
        refglobalpos.latitude = msg.latitude;
        refglobalpos.longitude = msg.longitude;
        //refglobalpos.altitude = msg.altitude;
				refglobalpos.altitude = msg.height;
        isGetrefglobalpos = true;

        ROS_INFO("Get reference global position successfully");
        ROS_INFO("Global position: %f, %f, %f, %f", msg.latitude, msg.longitude, msg.altitude, msg.height);
    }
}

void FlightControl::LocalPositionCallback(const dji_sdk::LocalPosition& msg)
{
    if(isStart && !isGetinitlocalpos)
    {
        initlocalpos.x = msg.x;
        initlocalpos.y = msg.y;
				initlocalpos.z = msg.z;
        isGetinitlocalpos = true;

        ROS_INFO("Get initial local position successfully");
        ROS_INFO("Local position: %f, %f, %f", msg.x, msg.y, msg.z);
    }
}

void FlightControl::AttitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion& msg)
{
    if(isStart && !isGetinitattitude)
    {
        double roll, pitch, yaw;
        tf::Quaternion quat(msg.q1, msg.q2, msg.q3, msg.q0);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        initattitude.roll = RAD2DEG(roll);
        initattitude.pitch = RAD2DEG(pitch);
        initattitude.yaw = RAD2DEG(yaw);
        isGetinitattitude = true;

        ROS_INFO("Get initial attitude successfully");
        ROS_INFO("Attitude: %f, %f, %f", initattitude.roll, initattitude.pitch, initattitude.yaw);
    }
}

void FlightControl::PublishFlightState(int8_t state)
{
    std_msgs::Int8 msg;
    msg.data = state;
    state_pub.publish(msg);

    //ROS_INFO("Publish state: %d", (int)state);
}

void FlightControl::CalcWaypoint()
{
    float x_min = xlim[0];
    float x_max = xlim[1];
    float y_min = ylim[0];
    float y_max = ylim[1];
    int nLine = round((x_max - x_min) / xstep);

    // Calculate local position of waypoint
    std::vector<ct::LocalPosition> bodywaypoints;
    ct::LocalPosition startpos;
    ct::LocalPosition endpos;
    ct::LocalPosition leftlocalpos;
    ct::LocalPosition rightlocalpos;
    startpos.x = 0;
    startpos.y = 0;
    startpos.z = height;
    endpos.x = nLine * xstep;
    endpos.y = 0;
    endpos.z = height;
    bodywaypoints.push_back(startpos); // add start point
    for(int i = 0;i <= nLine;i++)
    {
        leftlocalpos.x = i * xstep;
        leftlocalpos.y = y_min;
        leftlocalpos.z = height;
        rightlocalpos.x = i * xstep;
        rightlocalpos.y = y_max;
        rightlocalpos.z = height;
        if(i % 2)
        {
            bodywaypoints.push_back(rightlocalpos);
            bodywaypoints.push_back(leftlocalpos);
        }
        else
        {
            bodywaypoints.push_back(leftlocalpos);
            bodywaypoints.push_back(rightlocalpos);
        }
    }
    bodywaypoints.push_back(endpos); // add end point
    bodywaypoints.push_back(startpos);  // add start point again for landing

    // Calculate global position of waypoint
    ct::LocalPosition localpos;
    ct::GlobalPosition globalpos;
    for(int i = 0;i < bodywaypoints.size();i++)
    {
        ct::YawRotate(initattitude.yaw, bodywaypoints[i], localpos);
				localpos.x += initlocalpos.x;
				localpos.y += initlocalpos.y;
				localpos.z += initlocalpos.z;
        ct::LocalToGlocal(localpos, refglobalpos, globalpos);
        localwaypoints.push_back(localpos);
        globalwaypoints.push_back(globalpos);
    }
}

void FlightControl::ScanWaypoint()
{
    int num = localwaypoints.size();
    if(scanmode == 0)
    {
        ROS_INFO("scan mode 0");
        for(int i = 0;i < localwaypoints.size();i++)
        {
            double x = localwaypoints[i].x;
            double y = localwaypoints[i].y;
            double z = localwaypoints[i].z;
            ROS_INFO("Target position: %f, %f, %f", x, y, z);
            drone->local_position_navigation_wait_server();
            if(!drone->local_position_navigation_is_server_connected())
            {
                    ROS_ERROR("Local position navigation server isn't connected");
                    break;
            }
            drone->local_position_navigation_send_request(x, y, z);
            bool ret = drone->local_position_navigation_wait_for_result(ros::Duration(60));
            if(!ret)
            {
                    ROS_ERROR("Local position navigation fail");
                    break;
            }
        }
    }
    else if(scanmode == 1)
    {
        // Waypoint navigation
        ROS_INFO("scan mode 1");
        dji_sdk::WaypointList waypointList;
        dji_sdk::Waypoint waypoint;
        for(int i = 0;i < globalwaypoints.size();i++)
        {
            waypoint.latitude = globalwaypoints[i].latitude;
            waypoint.longitude = globalwaypoints[i].longitude;
            waypoint.altitude = globalwaypoints[i].altitude;
            waypoint.staytime = 0;
            waypoint.heading = initattitude.yaw;
            waypointList.waypoint_list.push_back(waypoint);
						ROS_INFO("waypoint: %d, %f, %f, %f", i + 1, waypoint.latitude, waypoint.longitude, waypoint.altitude);
        }
        drone->waypoint_navigation_wait_server();
        if(!drone->waypoint_navigation_is_server_connected() )
        {
                ROS_ERROR("Waypoint navigation server isn't connected");
        }
        drone->waypoint_navigation_send_request(waypointList);
        bool ret = drone->waypoint_navigation_wait_for_result(ros::Duration(num * 60));
        {
                ROS_ERROR("Waypoint navigation fail");
        }
    }
    else
    {
        ROS_INFO("scan mode 2");
        dji_sdk::MissionWaypointTask waypoint_task;
        dji_sdk::MissionWaypoint 	 waypoint;
        waypoint_task.velocity_range = velocity_range;
        waypoint_task.idle_velocity = idle_velocity; 
        waypoint_task.action_on_finish = 0;
        waypoint_task.mission_exec_times = 1;
        waypoint_task.yaw_mode = 1;
        /*!< 0: auto mode(point to next waypoint) <br>*/
        /*!< 1: lock as an initial value <br>*/
        /*!< 2: controlled by RC <br>*/
        /*!< 3: use waypoint's yaw(tgt_yaw) */
        waypoint_task.trace_mode = 0;
        waypoint_task.action_on_rc_lost = 0;
        waypoint_task.gimbal_pitch_mode = 0;
        /*!< 0: free mode, no control on gimbal <br>*/
        /*!< 1: auto mode, Smooth transition between waypoints <br>*/
        for(int i = 0;i < globalwaypoints.size();i++)
        {
            waypoint.latitude = globalwaypoints[i].latitude;
            waypoint.longitude = globalwaypoints[i].longitude;
            waypoint.altitude = globalwaypoints[i].altitude;
            waypoint.damping_distance = 0;
            waypoint.target_yaw = initattitude.yaw;
            waypoint.target_gimbal_pitch = 0;
            waypoint.turn_mode = 0;
            waypoint.has_action = 0;
            waypoint_task.mission_waypoint.push_back(waypoint);
					  ROS_INFO("waypoint: %d, %f, %f, %f", i + 1, waypoint.latitude, waypoint.longitude, waypoint.altitude);
        }
        drone->mission_waypoint_upload(waypoint_task);
        drone->mission_start();
        sleep(100);
    }
}

};




