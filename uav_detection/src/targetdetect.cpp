#include "targetdetect.h"


namespace target_detect{

bool TargetDetect::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    // Parameters
    state = S_NONE;
    nh_priv.param<std::string>("serial_port", serialconfig.port, "/dev/ttyUSB0");
    nh_priv.param<int>("serial_baudrate", serialconfig.baudrate, 115200);
    nh_priv.param<std::string>("Config_File", Config_File, "../params/detector_params.yml");
    nh_priv.param<std::string>("Param_File", Param_File, "../params/calibresult.txt");
    nh_priv.param<std::string>("Video_SaveFile", Video_SaveFile, "../result/result.avi");
    nh_priv.param<bool>("showimage", showimage, true);
    nh_priv.param<bool>("showresult", showresult, true);
    nh_priv.param<bool>("savevedio", savevedio, false);


    // Subscribe some topics
    globalposition_sub = nh.subscribe("dji_sdk/global_position", 10, &TargetDetect::GlobalPositionCallback, this);
    attitudequaternion_sub = nh.subscribe("dji_sdk/attitude_quaternion", 10, &TargetDetect::AttitudeQuaternionCallback, this);
    state_sub = nh.subscribe("state", 10, &TargetDetect::FlightStateCallback, this);

    // Start serial port
    if(!serialtransmission.StartSerial(serialconfig))
    {
        ROS_ERROR("Failed to start serial port");
        return false;
    }

    // Initial camera
    if(!camera.init())
    {
        ROS_ERROR("Failed to initial camera");
        return false;
    }

    // Initial detector
    if(!detector.init(Config_File, Param_File))
    {
        ROS_ERROR("Failed to initial detecter");
        return false;
    }
    sleep(10);

    ROS_INFO("Setup targetdetect node successfully");
    return true;
}

void TargetDetect::spin()
{
    ros::Rate loop_rate(20);

    // VideoWriter
    cv::VideoWriter videoWriter;
    if(savevedio)
    {
        videoWriter.open(Video_SaveFile, CV_FOURCC('M', 'P', '4', '2'), 30, cvSize(IMAGE_W, IMAGE_H));
        if(!videoWriter.isOpened())
        {
            ROS_ERROR("Failed to opencv videoWriter");
            return;
        }
    }

    cv::Mat img;
    std::vector<int> _ids;
    std::vector<ct::GlobalPosition> _globalpositions;
    while(ros::ok())
    {
        if(state == S_SCAN)
        {
            // Obtain image
            bool ret = camera.capture(img);
            if(ret && !img.empty())
            {
                if(showimage)
                {
										namedWindow("image", 0);
                    imshow("image", img);
                }
                // Target detection
                detector.detect(img, _ids);
                detector.drawmarker(img, Scalar(0, 0, 255), 10);
                if(showresult)
                {
                    imshow("result", img);
                }
                if(savevedio)
                {
                    videoWriter.write(img);
                }
                if(_ids.size() > 0)
                {
                    // Target location
                    detector.locate(attitude.yaw, -90.0, globalpos, _globalpositions);
                    // Save position data
                    for(int i = 0;i < _ids.size();i++)
                    {
                        ids.push_back(_ids[i]);
                        globalpositions.push_back(_globalpositions[i]);
                    }
                    _ids.clear();
                    _globalpositions.clear();
                }
            }
        }
        else if(state == S_ENDING)
        {
            break;
        }
				cv::waitKey(1);

        ros::spinOnce();
        loop_rate.sleep();
    }

    if(ids.size() > 0)
    {
        // Process position data
        ProcessPositionData();
        // Send position data to UGV
        SendPositionData();
    }
    else
    {
        ROS_INFO("There is no target detected");
    }

    // Exit
    camera.exit();
    videoWriter.release();
    ROS_INFO("Exit targetdetect node");
}

void TargetDetect::GlobalPositionCallback(const dji_sdk::GlobalPosition& msg)
{
    globalpos.latitude = msg.latitude;
    globalpos.longitude = msg.longitude;
    //globalpos.altitude = msg.altitude;
		globalpos.altitude = msg.height;

		//ROS_INFO("global position: %f, %f, %f", msg.latitude, msg.longitude, msg.altitude);
}

void TargetDetect::AttitudeQuaternionCallback(const dji_sdk::AttitudeQuaternion& msg)
{
    double roll, pitch, yaw;
    tf::Quaternion quat(msg.q1, msg.q2, msg.q3, msg.q0); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    attitude.roll = RAD2DEG(roll);
    attitude.pitch = RAD2DEG(pitch);
    attitude.yaw = RAD2DEG(yaw);

    //ROS_INFO("attitude: %f, %f, %f", attitude.roll, attitude.pitch, attitude.yaw);
}

void TargetDetect::FlightStateCallback(const std_msgs::Int8& msg)
{
    state = (FlightState)msg.data;
}

void TargetDetect::ProcessPositionData()
{
    //ROS_INFO("Process position data");

    int id;
    ct::GlobalPosition pos;
    int num;
    int accumnum[TAG_NUM];
    ct::GlobalPosition accumpos[TAG_NUM];

    // Initial accumulation array
    for(int i = 0;i < TAG_NUM;i++)
    {
        accumnum[i] = 0;
        accumpos[i].latitude = 0;
        accumpos[i].longitude = 0;
        accumpos[i].altitude = 0;
    }

    // Accumulate number and position
    for(int i = 0;i < ids.size();i++)
    {
        id = ids[i];
        pos = globalpositions[i];
        if(id < TAG_NUM)
        {
            accumnum[id]++;
            accumpos[id].latitude += pos.latitude;
            accumpos[id].longitude += pos.longitude;
            accumpos[id].altitude += pos.altitude;
        }
    }
    ids.clear();
    globalpositions.clear();

    // Average result of accumulation
    for(int i = 0;i < TAG_NUM;i++)
    {
        if(accumnum[i] > 0)
        {
            id = i;
            num = accumnum[i];
            pos.latitude = accumpos[i].latitude / num;
            pos.longitude = accumpos[i].longitude / num;
            pos.altitude = accumpos[i].altitude / num;
            ids.push_back(id);
            globalpositions.push_back(pos);
            ROS_INFO("id: %d, num: %d, lat, %f, lon, %f, alt, %f", id, num, pos.latitude, pos.longitude, pos.altitude);
        }
    }
}

void TargetDetect::SendPositionData()
{
    //ROS_INFO("Send position data");

    std::vector<float> data;
    ct::GlobalPosition pos;
    for(int i = 0;i < globalpositions.size();i++)
    {
        pos = globalpositions[i];
        data.push_back(pos.latitude);
        data.push_back(pos.longitude);
    }

    if(serialtransmission.SendDataWithSize(data))
    {
        ROS_INFO("Send position data successfully");
    }
    else
    {
        ROS_INFO("Failed to send position data");
    }
}

};




