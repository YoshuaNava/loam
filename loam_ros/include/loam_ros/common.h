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

#ifndef LOAM_ROS_COMMON_H
#define LOAM_ROS_COMMON_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>


namespace loam {


Eigen::Matrix4d getTransformFromQuaternion(const Eigen::Quaterniond q);

// const Eigen::Quaterniond rot_loam(0.7071, -0.7071, 0, 0);
const Eigen::Quaterniond rot_conv_loam(0.0005631, -0.0005631, 0.7071065, 0.7071065);
const Eigen::Quaterniond rot_fix_loam = Eigen::AngleAxisd(-1.5707, Eigen::Vector3d::UnitX())
                                  * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                  * Eigen::AngleAxisd(-1.5707, Eigen::Vector3d::UnitZ());
const Eigen::Matrix4d T_conv_loam = getTransformFromQuaternion(rot_conv_loam.inverse());
// const Eigen::Matrix4d T_fix_loam = getTransformFromQuaternion(rot_fix_loam.inverse());
// const Eigen::Matrix4d inv_T_fix_loam = T_fix_loam.inverse();
// const Eigen::Matrix4d inv_T_conv_loam = T_fix_loam.inverse();

/** \brief Construct a new point cloud message from the specified information and publish it via the given publisher.
 *
 * @tparam PointT the point type
 * @param publisher the publisher instance
 * @param cloud the cloud to publish
 * @param stamp the time stamp of the cloud message
 * @param frameID the message frame ID
 */
template <typename PointT>
inline void publishCloudMsg(ros::Publisher& publisher,
                            const pcl::PointCloud<PointT>& cloud,
                            const ros::Time& stamp,
                            std::string frameID) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = stamp;
  msg.header.frame_id = frameID;
  publisher.publish(msg);
}

// TODO doc
Eigen::Isometry3d convertOdometryToEigenIsometry(const nav_msgs::Odometry odom_msg);

// TODO doc
nav_msgs::Odometry convertEigenIsometryToOdometry(const std::string frame_id,
                                                  const Eigen::Isometry3d& odom,
                                                  const ros::Time stamp = ros::Time::now());

} // end namespace loam

#endif // LOAM_COMMON_H
