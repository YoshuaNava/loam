// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "loam_velodyne/TransformMaintenanceRos.h"

#include <fstream>


namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;


TransformMaintenanceRos::TransformMaintenanceRos()
{
  // initialize odometry and odometry tf messages
  _laserOdometry2.header.frame_id = "/camera_init";
  _laserOdometry2.child_frame_id = "/camera";

  _pathMsg.header.frame_id = "/camera_init";
  _laserOdometryTrans2.frame_id_ = "/camera_init";
  _laserOdometryTrans2.child_frame_id_ = "/camera";
}


bool TransformMaintenanceRos::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
  // advertise integrated laser odometry topic
  _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);

  _pubOdomToPath = node.advertise<nav_msgs::Path> ("/odom_to_path", 2);

  // subscribe to laser odometry and mapping odometry topics
  _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenanceRos::laserOdometryCallback, this);

  _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenanceRos::odomAftMappedCallback, this);

  return true;
}


void TransformMaintenanceRos::laserOdometryCallback(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  double[3] rpy;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transformSum[0] = -pitch;
  _transformSum[1] = -yaw;
  _transformSum[2] = roll;

  _transformSum[3] = laserOdometry->pose.pose.position.x;
  _transformSum[4] = laserOdometry->pose.pose.position.y;
  _transformSum[5] = laserOdometry->pose.pose.position.z;

  // transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      (_transformMapped[2], -_transformMapped[0], -_transformMapped[1]);

  _laserOdometry2.header.stamp = laserOdometry->header.stamp;
  _laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  _laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  _laserOdometry2.pose.pose.position.x = _transformMapped[3];
  _laserOdometry2.pose.pose.position.y = _transformMapped[4];
  _laserOdometry2.pose.pose.position.z = _transformMapped[5];
  _pubLaserOdometry2.publish(_laserOdometry2);

  _laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  _laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans2.setOrigin(tf::Vector3(_transformMapped[3], _transformMapped[4], _transformMapped[5]));
  _tfBroadcaster2.sendTransform(_laserOdometryTrans2);


  Eigen::Quaterniond quat(geoQuat.w, geoQuat.z, -geoQuat.x, -geoQuat.y);
  Eigen::Isometry3d T;
  T.linear() = quat.toRotationMatrix();
  T.translation() = Eigen::Vector3d(_transformMapped[3], _transformMapped[4], _transformMapped[5]);
  T = rot_kitti.toRotationMatrix() * T;

  publishPath(T.linear(), T.translation(), laserOdometry->header.stamp);
  savePoseToFile(T.linear(), T.translation());
}


void TransformMaintenanceRos::publishPath(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, const ros::Time stamp) {
  Eigen::Quaterniond q(rot);

  // set atributes of the msg
  geometry_msgs::PoseStamped _poseInPathMsg;
  _poseInPathMsg.header.stamp = stamp;
  _poseInPathMsg.pose.position.x = trans(0);
  _poseInPathMsg.pose.position.y = trans(1);
  _poseInPathMsg.pose.position.z = trans(2);
  _poseInPathMsg.pose.orientation.x = q.x();
  _poseInPathMsg.pose.orientation.y = q.y();
  _poseInPathMsg.pose.orientation.z = q.z();
  _poseInPathMsg.pose.orientation.w = q.w();
  _pathMsg.poses.push_back(_poseInPathMsg);
  _pathMsg.header.stamp = ros::Time::now();

  _pubOdomToPath.publish(_pathMsg);
}


void TransformMaintenanceRos::odomAftMappedCallback(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  _transformAftMapped[0] = -pitch;
  _transformAftMapped[1] = -yaw;
  _transformAftMapped[2] = roll;

  _transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  _transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  _transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

  _transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
  _transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
  _transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

  _transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
  _transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
  _transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}

} // end namespace loam
