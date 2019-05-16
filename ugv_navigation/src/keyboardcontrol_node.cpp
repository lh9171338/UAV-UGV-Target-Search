#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>
#include <stdio.h>


#define MOVE_LEFT 	0x44
#define MOVE_RIGHT 	0x43 
#define MOVE_FORWARD 	0x41
#define MOVE_BACKWARD 	0x42
#define TURN_LEFT	0x61
#define TURN_RIGHT	0x64
#define MOVE_UP		0x77
#define MOVE_DOWN	0x73


int kfd = 0;
struct termios cooked, raw;
ros::Publisher twist_pub;
double linear_x, linear_y, angular_z;
double l_scale, a_scale;
int rate;

void quit(int sig)
{
	(void)sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
}

void TimerCallback(const ros::TimerEvent&)
{
	//ROS_INFO("TimerCallback");
    
        // Get current key value and clear the buffer
	char ch, keyvalue = 0;
    while(read(kfd, &ch, 1) > 0){
		keyvalue = ch;
	}
	//ROS_INFO("value: 0x%x\n", keyvalue);

    linear_x = linear_y = angular_z = 0;
	switch(keyvalue)
	{
	case MOVE_LEFT:
		ROS_INFO("MOVE LEFT");
		linear_y = 1.0;
		break;
	case MOVE_RIGHT:
		ROS_INFO("MOVE RIGHT");
		linear_y = -1.0;
		break;
	case MOVE_FORWARD:
		ROS_INFO("MOVE FORWARD");
		linear_x = 1.0;
		break;
	case MOVE_BACKWARD:
		ROS_INFO("MOVE BACKWARD");
		linear_x = -1.0;
		break;
	case TURN_LEFT:
		ROS_INFO("TURN LEFT");
		angular_z = 1.0;
		break;
	case TURN_RIGHT:
		ROS_INFO("TURN RIGHT");
		angular_z = -1.0;
		break;
	case MOVE_UP:
		ROS_INFO("UP");
		// Add your code
		break;
	case MOVE_DOWN:
		ROS_INFO("DOWN");
		// Add your code
		break;
	}
   
	// Publish Twist message
	geometry_msgs::Twist twist;
	twist.linear.x = linear_x * l_scale;
	twist.linear.y = linear_y * l_scale;
	twist.angular.z = angular_z * a_scale;

	twist_pub.publish(twist);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "keyboardcontrol_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// Get parameters
	nh_priv.param<double>("l_scale", l_scale, 0.5);
	nh_priv.param<double>("a_scale", a_scale, 0.5);
    nh_priv.param<int>("rate", rate, 10);

	// Usage information
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move, 'a' and 'd' keys to turn");

	// Get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);                      
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	// Set read file in non-blocking mode
	int flags = fcntl(kfd, F_GETFL, 0); 
	flags |= O_NONBLOCK;         
	fcntl(kfd, F_SETFL, flags);       

	// Publish Twist message
	twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	// Create a timer
	ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), &TimerCallback);

        // Wait for ctrl'C to quit
	signal(SIGINT,quit);

	// Loop and execute callback function
	ros::spin();

	return 0;
}
