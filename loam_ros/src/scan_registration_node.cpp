#include <ros/ros.h>

#include "loam/ScanRegistrationRos.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::ScanRegistrationRos scanRegistrationRos;

  if(scanRegistrationRos.setup(node, privateNode))
    ros::spin();
  else {
    ROS_ERROR("Scan registration: Invalid parameters introduced. The node will shutdown.");
  }

  return 0;
}
