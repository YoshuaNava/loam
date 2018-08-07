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

#include "loam_ros/TransformMaintenanceRos.h"

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
  // load parameters
  _logResults = node.param<bool>("/loam/log_results", false);
  _logFilename = node.param<std::string>("/loam/log_filename", std::string("~/results.txt"));
  _publishPath = node.param<bool>("/loam/publish_path", false);
  
  // advertise integrated laser odometry topic
  _pubLaserOdometry2 = node.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
  _pubIntegratedOdometryFixed = node.advertise<nav_msgs::Odometry> ("/loam/fixed_integrated_to_init", 5);

  if(_publishPath)
    _pubOdomToPath = node.advertise<nav_msgs::Path> ("/odom_to_path", 2, true);

  // subscribe to laser odometry and mapping odometry topics
  _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &TransformMaintenanceRos::laserOdometryCallback, this);

  _subOdomAftMapped = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &TransformMaintenanceRos::odomAftMappedCallback, this);

  reset_service_ = node.advertiseService("transformMaintenance/reset", &TransformMaintenanceRos::resetCallback, this);
  pose_correction_service_ = node.advertiseService("transformMaintenance/correct_pose", &TransformMaintenanceRos::correctPoseCallback, this);
  return true;
}

bool TransformMaintenanceRos::resetCallback(std_srvs::Empty::Request  &req,
                                            std_srvs::Empty::Response &res)
{
  ROS_INFO("Transform maintenance RESET");
  _transformMaintainer.correctEstimate();
  return true;
}

bool TransformMaintenanceRos::correctPoseCallback(loam_msgs::PoseUpdate::Request  &req,
                                                  loam_msgs::PoseUpdate::Response &res)
{
  ROS_INFO("Transform maintenance CORRECTION");
  Eigen::Vector3d pos(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  Eigen::Quaterniond rot(req.pose.orientation.w, req.pose.orientation.z, -req.pose.orientation.x, -req.pose.orientation.y);
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0,1,2);
  res.success = true;

  // ROS_INFO("Received transform");
  // std::cout << "transform maintenance pose correction" << std::endl;
  // std::cout << "pos ->  " << pos.transpose() << std::endl;
  // std::cout << "rpy ->  " << rpy.transpose() << std::endl;
  // std::cout << "quat ->  " << rot.x() << ", " << rot.y() << ", " <<  rot.z() << ", " << rot.w() << std::endl;

  _transformMaintainer.correctEstimate(pos, rpy);

  // Eigen::Quaterniond rot(laserOdometry->pose.pose.orientation.w, laserOdometry->pose.pose.orientation.z, -laserOdometry->pose.pose.orientation.x, -laserOdometry->pose.pose.orientation.y);
  return true;
}

void TransformMaintenanceRos::laserOdometryCallback(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  Eigen::Vector3d pos(laserOdometry->pose.pose.position.x, laserOdometry->pose.pose.position.y,  laserOdometry->pose.pose.position.z);
  Eigen::Quaterniond rot(laserOdometry->pose.pose.orientation.w, laserOdometry->pose.pose.orientation.z, -laserOdometry->pose.pose.orientation.x, -laserOdometry->pose.pose.orientation.y);

  _transformMaintainer.processOdometryTransform(pos, rot);
  float* integratedTransform = _transformMaintainer.getIntegratedTransform();

  publishAndLogResults(integratedTransform, laserOdometry->header.stamp);
}

void TransformMaintenanceRos::odomAftMappedCallback(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{ 
  Eigen::Vector3d pos(odomAftMapped->pose.pose.position.x, odomAftMapped->pose.pose.position.y,  odomAftMapped->pose.pose.position.z);
  Eigen::Quaterniond rot(odomAftMapped->pose.pose.orientation.w, odomAftMapped->pose.pose.orientation.z, -odomAftMapped->pose.pose.orientation.x, -odomAftMapped->pose.pose.orientation.y);
  Eigen::Vector3d angular_vel(odomAftMapped->twist.twist.angular.x, odomAftMapped->twist.twist.angular.y,  odomAftMapped->twist.twist.angular.z);
  Eigen::Vector3d linear_vel(odomAftMapped->twist.twist.linear.x, odomAftMapped->twist.twist.linear.y,  odomAftMapped->twist.twist.linear.z);

  _transformMaintainer.processMappingTransform(pos, rot, linear_vel, angular_vel);
}

void TransformMaintenanceRos::publishAndLogResults(const float* transform, const ros::Time stamp)
{
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      (transform[2], -transform[0], -transform[1]);

  _laserOdometry2.header.stamp = stamp;
  _laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  _laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  _laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  _laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  _laserOdometry2.pose.pose.position.x = transform[3];
  _laserOdometry2.pose.pose.position.y = transform[4];
  _laserOdometry2.pose.pose.position.z = transform[5];
  _pubLaserOdometry2.publish(_laserOdometry2);

  // Integrated odometry message in XYZ coordinate frame
  Eigen::Isometry3d T_odom = convertOdometryToEigenIsometry(_laserOdometry2);
  T_odom = rot_conv_loam.toRotationMatrix().inverse() * T_odom;
  T_odom.linear() *= rot_fix_loam.toRotationMatrix();
  _integratedOdometryFixed = convertEigenIsometryToOdometry("/camera_init", T_odom, stamp);
  _pubIntegratedOdometryFixed.publish(_integratedOdometryFixed);

  _laserOdometryTrans2.stamp_ = stamp;
  _laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  _laserOdometryTrans2.setOrigin(tf::Vector3(transform[3], transform[4], transform[5]));
  _tfBroadcaster2.sendTransform(_laserOdometryTrans2);

  Eigen::Quaterniond quat = Eigen::AngleAxisd(transform[2], Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(-transform[0], Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(-transform[1], Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat_kitti(quat.w(), -quat.y(), -quat.z(), quat.x());
  Eigen::Isometry3d T;
  T.linear() = quat_kitti.toRotationMatrix();
  T.translation() = Eigen::Vector3d(transform[3], transform[4], transform[5]);
  T = rot_kitti.toRotationMatrix() * T;

  if(_logResults)
    savePoseToFile(T.linear(), T.translation(), _logFilename);

  if(_publishPath)
    publishPath(T.linear(), T.translation(), stamp);
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

} // end namespace loam
