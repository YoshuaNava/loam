#include <ros/ros.h>
#include "loam_velodyne/CtRot2DScanRegistration.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::CtRot2DScanRegistration ctRot2DScan;

  if (ctRot2DScan.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
