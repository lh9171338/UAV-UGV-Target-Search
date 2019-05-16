#include <ros/ros.h>
#include "targetdetect.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "targetdetect_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create TargetDetect class object
  target_detect::TargetDetect targetdetect;

  if(targetdetect.setup(nh, nh_priv))
  {
    // Initial targetdetect successfully
    targetdetect.spin();
  }

  return 0;
}
