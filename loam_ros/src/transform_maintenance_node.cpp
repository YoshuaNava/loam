#include <ros/ros.h>
#include "loam_velodyne/TransformMaintenanceRos.h"


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::TransformMaintenanceRos transMaintenanceRos;

  if (transMaintenanceRos.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
