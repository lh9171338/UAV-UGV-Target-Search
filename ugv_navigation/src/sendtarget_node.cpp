#include <ros/ros.h>
#include "sendtarget.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sendtarget_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create SendTarget class object
  send_target::SendTarget sendtarget;

  sendtarget.setup(nh, nh_priv);

  return 0;
}
