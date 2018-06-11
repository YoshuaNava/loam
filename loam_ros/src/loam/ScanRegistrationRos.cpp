#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include "loam/ScanRegistrationRos.h"


namespace loam {

bool ScanRegistrationRos::setup(ros::NodeHandle& node,
                                ros::NodeHandle& privateNode) {

  loam::ScanRegistrationParams params = loadParameters(node, privateNode);

  if((params.lidarModel == "VLP-16") || (params.lidarModel == "HDL-32") 
     || (params.lidarModel == "HDL-64E") || (params.lidarModel == "velodyne")
     || (params.lidarModel == "linear"))
    _scanRegistration.reset(new MultiScanRegistration(params));
  else if (params.lidarModel == "continuous")
    _scanRegistration.reset(new CtScanRegistration(params));
  // else if (params.lidarModel == "back_and_forth")
  //   _scanRegistration.reset(new CtScanRegistration(params));
  else
    return false;

  // advertise scan registration topics
  _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 2);
  _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  _pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
        ("/multi_scan_points", 2, &ScanRegistrationRos::handleCloudMessage, this);

  // // subscribe to IMU topic
  _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistrationRos::handleIMUMessage, this);

  return true;
}


loam::ScanRegistrationParams ScanRegistrationRos::loadParameters(ros::NodeHandle& node,
                                                                 ros::NodeHandle& privateNode)
{
  loam::ScanRegistrationParams params = loam::ScanRegistrationParams();
  const char* module_name = "MultiScanRegistration";
  
  // fetch laser mapping params
  float fParam;
  int iParam;

  if (node.getParam("/loam/scan_period", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid scan_period parameter: %f (expected > 0)", module_name, fParam);
    } else {
      params.scanPeriod = fParam;
      // ROS_INFO("%s: Set scan_period: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/imu_history_size", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid imu_history_size parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.imuHistorySize = iParam;
      // ROS_INFO("%s: Set imu_history_size: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/n_feature_regions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid n_feature_regions parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.nFeatureRegions = iParam;
      // ROS_INFO("%s: Set n_feature_regions: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/curvature_region", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid curvature_region parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.curvatureRegion = iParam;
      // ROS_INFO("%s: Set curvature_region: +/- %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_corner_sharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_corner_sharp parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.maxCornerSharp = iParam;
      params.maxCornerLessSharp = 10 * iParam;
      // ROS_INFO("%s: Set max_corner_sharp / less sharp: %d / %d", module_name, iParam, params.maxCornerLessSharp);
    }
  }

  if (node.getParam("/loam/registration/max_corner_less_sharp", iParam)) {
    if (iParam < params.maxCornerSharp) {
      ROS_ERROR("%s: Invalid max_corner_less_sharp parameter: %d (expected >= %d)", module_name, iParam, params.maxCornerSharp);
    } else {
      params.maxCornerLessSharp = iParam;
      // ROS_INFO("%s: Set max_corner_less_sharp: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_surface_flat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_surface_flat parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.maxSurfaceFlat = iParam;
      // ROS_INFO("%s: Set max_surface_flat: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/surface_curvature_threshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid surface_curvature_threshold parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.surfaceCurvatureThreshold = fParam;
      // ROS_INFO("%s: Set surface_curvature_threshold: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/less_flat_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid less_flat_filter_size parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.lessFlatFilterSize = fParam;
      // ROS_INFO("%s: Set less_flat_filter_size: %g", module_name, fParam);
    }
  }

  // load MultiScanMapper params
  bool validParams = true;
  if (node.getParam("/loam/registration/lidar_model", params.lidarModel)) {
    if((params.lidarModel == "VLP-16") || (params.lidarModel == "HDL-32") || (params.lidarModel == "HDL-64E"))
      ROS_INFO("%s: Set  %s  scan mapper.", module_name, params.lidarModel.c_str());
    else
      validParams = false;
  }
  else if (params.lidarModel == "linear" &&
      node.getParam("/loam/registration/min_vertical_angle", params.vAngleMin) &&
      node.getParam("/loam/registration/max_vertical_angle", params.vAngleMax) &&
      node.getParam("/loam/registration/n_scan_rings", params.nScanRings)) {
    if (params.vAngleMin >= params.vAngleMax) {
      ROS_ERROR("%s: Invalid vertical range (min >= max)", module_name);
      validParams = false;
    } else if (params.nScanRings < 2) {
      ROS_ERROR("%s: Invalid number of scan rings (n < 2)", module_name);
      validParams = false;
    }
    ROS_INFO("%s: Set linear scan mapper from %g to %g degrees with %d scan rings.", module_name, params.vAngleMin, params.vAngleMax, params.nScanRings);
  } else if (params.lidarModel == "continuous") {
    validParams = true;
  }

  if (!validParams) {
    ROS_ERROR("%s: Invalid scan registration parameters", module_name);
    ROS_ERROR("%s: Default VLP-16 registration model will be used", module_name);
    params.lidarModel = "VLP-16";
  }

  return params;
}


/** \brief Handler method for IMU messages.
 *
 * @param imuIn the new IMU message
 */
void ScanRegistrationRos::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  loam::Vector3 acc;
  acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  acc.z() = float(imuIn->linear_acceleration.x + sin(pitch) * 9.81);

  loam::IMUState newState;
  newState.stamp = imuIn->header.stamp.toSec();
  newState.roll = roll;
  newState.pitch = pitch;
  newState.yaw = yaw;
  newState.acceleration = acc;

  _scanRegistration->addImuData(newState);
}


/** \brief Handler method for input cloud messages.
 *
 * @param laserCloudMsg the new input cloud message to process
 */
void ScanRegistrationRos::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  if(_scanRegistration->process(laserCloudIn, laserCloudMsg->header.stamp.toSec()))
    publishResults();
}


/** \brief Publish the current results via the respective topics. */
void ScanRegistrationRos::publishResults()
{
  loam::Time sweepStart = _scanRegistration->sweepStart();
  loam::IMUState imuStart = _scanRegistration->imuStart();
  loam::IMUState imuCur = _scanRegistration->imuCur();
  loam::Vector3 imuShiftFromStart = _scanRegistration->imuPositionShift();
  loam::Vector3 imuVelocityFromStart = imuCur.velocity - imuStart.velocity;
  pcl::PointCloud<pcl::PointXYZ> imuTrans(4,1);


  // publish full resolution and feature point clouds
  loam::publishCloudMsg(_pubLaserCloud,            _scanRegistration->laserCloud(),            ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(_pubCornerPointsSharp,     _scanRegistration->cornerPointsSharp(),     ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(_pubCornerPointsLessSharp, _scanRegistration->cornerPointsLessSharp(), ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(_pubSurfPointsFlat,        _scanRegistration->surfacePointsFlat(),     ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(_pubSurfPointsLessFlat,    _scanRegistration->surfacePointsLessFlat(), ros::Time(sweepStart), "/camera");


  // publish corresponding IMU transformation information
  imuTrans[0].x = imuStart.pitch.rad();
  imuTrans[0].y = imuStart.yaw.rad();
  imuTrans[0].z = imuStart.roll.rad();

  imuTrans[1].x = imuCur.pitch.rad();
  imuTrans[1].y = imuCur.yaw.rad();
  imuTrans[1].z = imuCur.roll.rad();

  loam::rotateYXZ(imuShiftFromStart, -imuStart.yaw, -imuStart.pitch, -imuStart.roll);

  imuTrans[2].x = imuShiftFromStart.x();
  imuTrans[2].y = imuShiftFromStart.y();
  imuTrans[2].z = imuShiftFromStart.z();

  loam::rotateYXZ(imuVelocityFromStart, -imuStart.yaw, -imuStart.pitch, -imuStart.roll);

  imuTrans[3].x = imuVelocityFromStart.x();
  imuTrans[3].y = imuVelocityFromStart.y();
  imuTrans[3].z = imuVelocityFromStart.z();

  loam::publishCloudMsg(_pubImuTrans, imuTrans, ros::Time(sweepStart), "/camera");
}


} // end namespace loam