#include <ros/ros.h>
#include "flightcontrol.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "flightcontrol_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Create FlightControl class object
  flight_control::FlightControl flightcontrol;

  if(flightcontrol.setup(nh, nh_priv))
  {
    // Initial flightcontrol successfully
    flightcontrol.spin();
  }

  return 0;
}
