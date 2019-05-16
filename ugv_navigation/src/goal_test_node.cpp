#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>


using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
#define NUM_TARGETS	5


// Target points
vector<float> targets[NUM_TARGETS];
string fix_frame;
int num_target;
int executetime;
int sleeptime;

void setGoal(
	vector<float>&					target,
	move_base_msgs::MoveBaseGoal&	goal
	)
{
	goal.target_pose.header.frame_id = fix_frame;
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = target[0];
	goal.target_pose.pose.position.y = target[1];
	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target[2]);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sendgoal_node");
	ros::NodeHandle nh_priv("~");

    	// Get parameters
   	nh_priv.param<string>("fix_frame", fix_frame, "map");
   	nh_priv.param<int>("executetime", executetime, 100);
   	nh_priv.param<int>("sleeptime", sleeptime, 5);
	nh_priv.param<int>("num_target", num_target, 5);
	nh_priv.getParam("P1", targets[0]);
	nh_priv.getParam("P2", targets[1]);
	nh_priv.getParam("P3", targets[2]);
	nh_priv.getParam("P4", targets[3]);
	nh_priv.getParam("P5", targets[4]);

	move_base_msgs::MoveBaseGoal goal;

	MoveBaseClient clinet("move_base", true);
	clinet.waitForServer();

	for(int i = 0;i < num_target;i++)
	{
		setGoal(targets[i], goal);
		clinet.sendGoal(goal);
		clinet.waitForResult(ros::Duration(executetime));
		if(clinet.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("Failed to move to the target");
			break;
		}
		sleep(sleeptime);
	}

	return 0;
}
