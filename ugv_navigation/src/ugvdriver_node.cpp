#include <ros/ros.h>
#include "ugvdriver.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ugvdriver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create UgvNavigation class object
  ugv_driver::UgvDriver ugvdriver;

  if(ugvdriver.setup(nh, nh_priv))
  {
     // Initial ugvnavigation successfully
    ugvdriver.spin();
  }

  return 0;
}
