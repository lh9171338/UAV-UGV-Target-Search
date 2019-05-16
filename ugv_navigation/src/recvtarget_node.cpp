#include <ros/ros.h>
#include "recvtarget.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "recvtarget_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create RecvTarget class object
  recv_target::RecvTarget recvtarget;

  recvtarget.setup(nh, nh_priv);

  return 0;
}
