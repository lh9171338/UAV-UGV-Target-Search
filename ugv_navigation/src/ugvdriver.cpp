#include "ugvdriver.h"


namespace ugv_driver{

bool UgvDriver::setup(ros::NodeHandle &nh, ros::NodeHandle &nh_priv)
{
    isfirst = true;

    // Parameters
    nh_priv.param<string>("serial_port", serialconfig.port, "/dev/ttyUSB0");
    nh_priv.param<int>("serial_baudrate", serialconfig.baudrate, 115200);
    nh_priv.param<string>("odom_frame", odom_frame, "odom");
    nh_priv.param<string>("base_frame", base_frame, "base_link");
    nh_priv.param<bool>("pub_base_odom_transform", pub_base_odom_transform, true);
    nh_priv.param<int>("odom_rate", odom_rate, 10);
    nh_priv.param<int>("imu_accel_range", imu_accel_range, 8);
    nh_priv.param<int>("imu_gyro_range", imu_gyro_range, 2000);

    // Subscribe the cmd_vel topic
    vel_sub = nh.subscribe("cmd_vel", 10, &UgvDriver::VelCallback, this);

    // Create publisher
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);

    // Start serial
    if(!serialtransmission.StartSerial(serialconfig))
    {
        return false;
    }

    return true;
}

void UgvDriver::spin()
{
    // Loop rate
    ros::Rate loop_rate(odom_rate);

    while(ros::ok())
    {
        if(RecvSensorData())
        {
            CalcOdometry();
            PublishSensorData();
        }

        // Loop and execute callback function
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void UgvDriver::VelCallback(const geometry_msgs::Twist& msg)
{
    //ROS_INFO("VelCallback");

    vel_twist = msg;
    SendVelData();
}

void UgvDriver::CalcOdometry()
{
    double vx = wheel_odom.twist.twist.linear.x;
    double vy = wheel_odom.twist.twist.linear.y;
    double vth = imu.angular_velocity.z;

    last_time = current_time;
    current_time = ros::Time::now();

    if(isfirst)
    {
        isfirst = false;

        pose.x = 0;
        pose.y = 0;
 //       pose.theta = 0;
        pose.theta = yaw;
    }
    else
    {
//        double dt = (current_time - last_time).toSec();
//        double dx = vx * dt;
//        double dy = vy * dt;
//        double dth = vth * dt;
//        double delta_x = dx * cos(pose.theta) - dy * sin(pose.theta);
//        double delta_y = dx * sin(pose.theta) + dy * cos(pose.theta);
//        pose.x += delta_x;
//        pose.y += delta_y;
//        pose.theta += dth;
//        if(pose.theta >= C_PI)
//        {
//            pose.theta -= 2 * C_PI;
//        }
//        else if(pose.theta <= -C_PI)
//        {
//            pose.theta += 2 * C_PI;
//        }

        double dx = wheel_odom.pose.pose.position.x - last_wheel_odom.pose.pose.position.x;
        double dy = wheel_odom.pose.pose.position.y - last_wheel_odom.pose.pose.position.y;

        double delta_x = dx * cos(pose.theta) - dy * sin(pose.theta);
        double delta_y = dx * sin(pose.theta) + dy * cos(pose.theta);
        pose.x += delta_x;
        pose.y += delta_y;
        pose.theta = yaw;
    }
    //ROS_INFO("%lf, %lf, %lf, %lf, %lf, %lf", vx, vy, vth, pose.x, pose.y, RAD2DEG(pose.theta));
}

void UgvDriver::PublishSensorData()
{
    double x = pose.x;
    double y = pose.y;
    double vx = wheel_odom.twist.twist.linear.x;
    double vy = wheel_odom.twist.twist.linear.y;
    double vth = imu.angular_velocity.z;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pose.theta);

    // Send the transform
    if(pub_base_odom_transform)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odom_frame;
        odom_trans.child_frame_id = base_frame;

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = quat;

        odom_broadcaster.sendTransform(odom_trans);
    }

    // Publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quat;

    odom.child_frame_id = base_frame;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);

    // Publish IMU
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = base_frame;

    imu_pub.publish(imu);

    // Publish GPS
    sensor_msgs::NavSatFix gps;
    gps.header.stamp = ros::Time::now();
    gps.header.frame_id = odom_frame;
    gps.latitude = globalpos.latitude;
    gps.longitude = globalpos.longitude;
    gps.altitude = globalpos.altitude;

    gps_pub.publish(gps);
}

bool UgvDriver::RecvSensorData()
{
    short data[20];

    // Receive data
    if(!serialtransmission.RecvData(data, 20))
    {
        return false;
    }

    // IMU data
    imu.linear_acceleration.x = data[0] / 32768.0 * imu_accel_range * 9.8; // unit: m/s^2
    imu.linear_acceleration.y = data[1] / 32768.0 * imu_accel_range * 9.8;
    imu.linear_acceleration.z = data[2] / 32768.0 * imu_accel_range * 9.8;

    imu.angular_velocity.x = DEG2RAD(data[3] / 32768.0 * imu_gyro_range); // unit: rad
    imu.angular_velocity.y = DEG2RAD(data[4] / 32768.0 * imu_gyro_range);
    imu.angular_velocity.z = DEG2RAD(data[5] / 32768.0 * imu_gyro_range);

    roll = DEG2RAD(data[6] / 32768.0 * 180.0); // unit: rad
    pitch = DEG2RAD(data[7] / 32768.0 * 180.0);
    yaw = DEG2RAD(data[8] / 32768.0 * 180.0);
    imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Magnetometer data
    magnetometer.magnetic_field.x = data[9]; // unit: mT
    magnetometer.magnetic_field.y = data[10];
    magnetometer.magnetic_field.z = data[11];

    // GPS data
    int32_t lon = (data[12] | data[13] << 16);
    int32_t lat = (data[14] | data[15] << 16);

    globalpos.longitude = (double)(lon / 10000000) + ((double)(lon % 10000000) / 100000.0) / 60.0; // unit: degree
    globalpos.latitude = (double)(lat / 10000000) + ((double)(lat % 10000000) / 100000.0) / 60.0;
    globalpos.altitude = 0;

    // Wheel odometry data
    last_wheel_odom.pose.pose.position.x = wheel_odom.pose.pose.position.x;
    last_wheel_odom.pose.pose.position.y = wheel_odom.pose.pose.position.y;
    wheel_odom.pose.pose.position.x = data[16] / 100.0; // unit: m
    wheel_odom.pose.pose.position.y = data[17] / 100.0;
    wheel_odom.twist.twist.linear.x = data[18] / 10000.0; // unit: m/s
    wheel_odom.twist.twist.linear.y = data[19] / 10000.0; // unit: m/s

    // Debug
    //ROS_INFO("linear_acceleration: %f, %f, %f", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    //ROS_INFO("angular_velocity: %f, %f, %f", imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
    ROS_INFO("attitude: %f, %f, %f", RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw));
    //ROS_INFO("magnetometer: %f, %f, %f", magnetometer.magnetic_field.x, magnetometer.magnetic_field.y, magnetometer.magnetic_field.z);
    //ROS_INFO("globalposition: %f, %f, %f", globalpos.latitude, globalpos.longitude, globalpos.altitude);
    //ROS_INFO("wheel_odom: %f, %f, %f, %f", wheel_odom.pose.pose.position.x, wheel_odom.pose.pose.position.y,
    //         wheel_odom.twist.twist.linear.x, wheel_odom.twist.twist.linear.y);

    return true;
}

bool UgvDriver::SendVelData()
{
    short data[3];

    // Set data
    data[0] = round(vel_twist.linear.x * 10000.0);
    data[1] = round(vel_twist.linear.y * 10000.0);
    data[2] = round(vel_twist.angular.z * 10000.0);
    //ROS_INFO("%f, %f, %f", vel_twist.linear.x, vel_twist.linear.y, vel_twist.angular.z);

    // Send data
    return serialtransmission.SendData(data, 3);
}

};




