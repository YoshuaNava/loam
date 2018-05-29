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

#include "loam_velodyne/ScanRegistration.h"
#include "loam_utils/math_utils.h"

#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>


namespace loam {

bool ScanRegistration::setup(ros::NodeHandle& node,
                             ros::NodeHandle& privateNode)
{
  const char* module_name = "ScanRegistration";
  
  // fetch laser mapping params
  float fParam;
  int iParam;

  if (node.getParam("/loam/scan_period", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid scan_period parameter: %f (expected > 0)", module_name, fParam);
    } else {
      _params.scanPeriod = fParam;
      ROS_INFO("%s: Set scan_period: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/imu_history_size", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid imu_history_size parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      _params.imuHistorySize = iParam;
      ROS_INFO("%s: Set imu_history_size: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/n_feature_regions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid n_feature_regions parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      _params.nFeatureRegions = iParam;
      ROS_INFO("%s: Set n_feature_regions: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/curvature_region", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid curvature_region parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      _params.curvatureRegion = iParam;
      ROS_INFO("%s: Set curvature_region: +/- %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_corner_sharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_corner_sharp parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      _params.maxCornerSharp = iParam;
      _params.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("%s: Set max_corner_sharp / less sharp: %d / %d", module_name, iParam, _params.maxCornerLessSharp);
    }
  }

  if (node.getParam("/loam/registration/max_corner_less_sharp", iParam)) {
    if (iParam < _params.maxCornerSharp) {
      ROS_ERROR("%s: Invalid max_corner_less_sharp parameter: %d (expected >= %d)", module_name, iParam, _params.maxCornerSharp);
    } else {
      _params.maxCornerLessSharp = iParam;
      ROS_INFO("%s: Set max_corner_less_sharp: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_surface_flat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_surface_flat parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      _params.maxSurfaceFlat = iParam;
      ROS_INFO("%s: Set max_surface_flat: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/surface_curvature_threshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid surface_curvature_threshold parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      _params.surfaceCurvatureThreshold = fParam;
      ROS_INFO("%s: Set surface_curvature_threshold: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/less_flat_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid less_flat_filter_size parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      _params.lessFlatFilterSize = fParam;
      ROS_INFO("%s: Set less_flat_filter_size: %g", module_name, fParam);
    }
  }

  _imuHistory.ensureCapacity(_params.imuHistorySize);

  // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);


  // advertise scan registration topics
  _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 2);
  _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}



void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  IMUState newState;
  newState.stamp = imuIn->header.stamp.toSec();
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(acc, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last();
    float timeDiff = float(newState.stamp - prevState.stamp);
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * acc * timeDiff * timeDiff);
    newState.velocity = prevState.velocity
                        + acc * timeDiff;
  }

  _imuHistory.push(newState);
}


void ScanRegistration::publishResult()
{
  // publish full resolution and feature point clouds
  publishCloudMsg(_pubLaserCloud,            _laserCloud,            ros::Time(_sweepStart), "/camera");
  publishCloudMsg(_pubCornerPointsSharp,     _cornerPointsSharp,     ros::Time(_sweepStart), "/camera");
  publishCloudMsg(_pubCornerPointsLessSharp, _cornerPointsLessSharp, ros::Time(_sweepStart), "/camera");
  publishCloudMsg(_pubSurfPointsFlat,        _surfacePointsFlat,     ros::Time(_sweepStart), "/camera");
  publishCloudMsg(_pubSurfPointsLessFlat,    _surfacePointsLessFlat, ros::Time(_sweepStart), "/camera");


  // publish corresponding IMU transformation information
  _imuTrans[0].x = _imuStart.pitch.rad();
  _imuTrans[0].y = _imuStart.yaw.rad();
  _imuTrans[0].z = _imuStart.roll.rad();

  _imuTrans[1].x = _imuCur.pitch.rad();
  _imuTrans[1].y = _imuCur.yaw.rad();
  _imuTrans[1].z = _imuCur.roll.rad();

  Vector3 imuShiftFromStart = _imuPositionShift;
  rotateYXZ(imuShiftFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[2].x = imuShiftFromStart.x();
  _imuTrans[2].y = imuShiftFromStart.y();
  _imuTrans[2].z = imuShiftFromStart.z();

  Vector3 imuVelocityFromStart = _imuCur.velocity - _imuStart.velocity;
  rotateYXZ(imuVelocityFromStart, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);

  _imuTrans[3].x = imuVelocityFromStart.x();
  _imuTrans[3].y = imuVelocityFromStart.y();
  _imuTrans[3].z = imuVelocityFromStart.z();

  publishCloudMsg(_pubImuTrans, _imuTrans, ros::Time(_sweepStart), "/camera");
}

} // end namespace loam
