
#include "loam_ros/common.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>


namespace loam {


Eigen::Matrix4d getTransformFromQuaternion(const Eigen::Quaterniond q) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = q.toRotationMatrix();
  return T;
}

Eigen::Isometry3d convertOdometryToEigenIsometry(const nav_msgs::Odometry odom_msg) {
  const auto& orientation = odom_msg.pose.pose.orientation;
  const auto& position = odom_msg.pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  
  return isometry;
}

nav_msgs::Odometry convertEigenIsometryToOdometry(const std::string frame_id,
                                                  const Eigen::Isometry3d& odom,
                                                  const ros::Time stamp) {
  nav_msgs::Odometry odom_msg;

  Eigen::Vector3d pos = odom.translation();
  Eigen::Quaterniond rot(odom.rotation());

  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id;
  odom_msg.pose.pose.position.x = pos.x();
  odom_msg.pose.pose.position.y = pos.y();
  odom_msg.pose.pose.position.z = pos.z();
  odom_msg.pose.pose.orientation.x = rot.x();
  odom_msg.pose.pose.orientation.y = rot.y();
  odom_msg.pose.pose.orientation.z = rot.z();
  odom_msg.pose.pose.orientation.w = rot.w();

  return odom_msg;
}


} // end namespace loam
