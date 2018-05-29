#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include "loam_utils/math_utils.h"
#include "loam_velodyne/MultiScanRegistration.h"


std::unique_ptr<loam::MultiScanRegistration> multiScanRegistration;

ros::Subscriber subLaserCloud;   ///< input cloud message subscriber
ros::Subscriber subImu;    ///< IMU message subscriber

ros::Publisher pubLaserCloud;              ///< full resolution cloud message publisher
ros::Publisher pubCornerPointsSharp;       ///< sharp corner cloud message publisher
ros::Publisher pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher
ros::Publisher pubSurfPointsFlat;          ///< flat surface cloud message publisher
ros::Publisher pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher
ros::Publisher pubImuTrans;                ///< IMU transformation message publisher


/** \brief Publish the current results via the respective topics. */
void publishResults()
{
  loam::Time sweepStart = multiScanRegistration->sweepStart();
  loam::IMUState imuStart = multiScanRegistration->imuStart();
  loam::IMUState imuCur = multiScanRegistration->imuCur();
  loam::Vector3 imuShiftFromStart = multiScanRegistration->imuPositionShift();
  loam::Vector3 imuVelocityFromStart = imuCur.velocity - imuStart.velocity;
  pcl::PointCloud<pcl::PointXYZ> imuTrans(4,1);


  // publish full resolution and feature point clouds
  loam::publishCloudMsg(pubLaserCloud,            multiScanRegistration->laserCloud(),            ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(pubCornerPointsSharp,     multiScanRegistration->cornerPointsSharp(),     ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(pubCornerPointsLessSharp, multiScanRegistration->cornerPointsLessSharp(), ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(pubSurfPointsFlat,        multiScanRegistration->surfacePointsFlat(),     ros::Time(sweepStart), "/camera");
  loam::publishCloudMsg(pubSurfPointsLessFlat,    multiScanRegistration->surfacePointsLessFlat(), ros::Time(sweepStart), "/camera");


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

  loam::publishCloudMsg(pubImuTrans, imuTrans, ros::Time(sweepStart), "/camera");
}



/** \brief Handler method for IMU messages.
 *
 * @param imuIn the new IMU message
 */
void handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
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

  multiScanRegistration->addImuData(newState);
}



/** \brief Handler method for input cloud messages.
 *
 * @param laserCloudMsg the new input cloud message to process
 */
void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (multiScanRegistration->systemDelay() > 0) {
    multiScanRegistration->systemDelay() --;
    return;
  }

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  multiScanRegistration->process(laserCloudIn, laserCloudMsg->header.stamp.toSec());

  publishResults();
}


loam::ScanRegistrationParams loadParameters(ros::NodeHandle& node,
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
      ROS_INFO("%s: Set scan_period: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/imu_history_size", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid imu_history_size parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.imuHistorySize = iParam;
      ROS_INFO("%s: Set imu_history_size: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/n_feature_regions", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid n_feature_regions parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.nFeatureRegions = iParam;
      ROS_INFO("%s: Set n_feature_regions: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/curvature_region", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid curvature_region parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.curvatureRegion = iParam;
      ROS_INFO("%s: Set curvature_region: +/- %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_corner_sharp", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_corner_sharp parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.maxCornerSharp = iParam;
      params.maxCornerLessSharp = 10 * iParam;
      ROS_INFO("%s: Set max_corner_sharp / less sharp: %d / %d", module_name, iParam, params.maxCornerLessSharp);
    }
  }

  if (node.getParam("/loam/registration/max_corner_less_sharp", iParam)) {
    if (iParam < params.maxCornerSharp) {
      ROS_ERROR("%s: Invalid max_corner_less_sharp parameter: %d (expected >= %d)", module_name, iParam, params.maxCornerSharp);
    } else {
      params.maxCornerLessSharp = iParam;
      ROS_INFO("%s: Set max_corner_less_sharp: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/max_surface_flat", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_surface_flat parameter: %d (expected >= 1)", module_name, iParam);
    } else {
      params.maxSurfaceFlat = iParam;
      ROS_INFO("%s: Set max_surface_flat: %d", module_name, iParam);
    }
  }

  if (node.getParam("/loam/registration/surface_curvature_threshold", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid surface_curvature_threshold parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.surfaceCurvatureThreshold = fParam;
      ROS_INFO("%s: Set surface_curvature_threshold: %g", module_name, fParam);
    }
  }

  if (node.getParam("/loam/registration/less_flat_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid less_flat_filter_size parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.lessFlatFilterSize = fParam;
      ROS_INFO("%s: Set less_flat_filter_size: %g", module_name, fParam);
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
  }

  if (!validParams) {
    ROS_ERROR("%s: Invalid scan registration parameters", module_name);
    ROS_ERROR("%s: Default VLP-16 registration model will be used", module_name);
    params.lidarModel = "VLP-16";
  }

  return params;
}

loam::MultiScanMapper createScanMapper(loam::ScanRegistrationParams params)
{
  loam::MultiScanMapper scanMapper = loam::MultiScanMapper();

  // fetch scan matching params
  if (params.lidarModel != "linear" && params.lidarModel != "none") {
    if (params.lidarModel == "VLP-16") {
      scanMapper = loam::MultiScanMapper::Velodyne_VLP_16();
    } else if (params.lidarModel == "HDL-32") {
      scanMapper = loam::MultiScanMapper::Velodyne_HDL_32();
    } else if (params.lidarModel == "HDL-64E") {
      scanMapper = loam::MultiScanMapper::Velodyne_HDL_64E();
    }
  } else if (params.lidarModel == "linear") {
    scanMapper.set(params.vAngleMin, params.vAngleMax, params.nScanRings);
  } else {
    scanMapper = loam::MultiScanMapper::Velodyne_VLP_16();
  }

  return scanMapper;
}


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::ScanRegistrationParams params = loadParameters(node, privateNode);
  loam::MultiScanMapper scanMapper = createScanMapper(params);
  multiScanRegistration.reset(new loam::MultiScanRegistration(scanMapper, params));


  // // advertise scan registration topics
  pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 2);
  pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  pubImuTrans = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  // subscribe to input cloud topic
  subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/multi_scan_points", 2, handleCloudMessage);

  // // subscribe to IMU topic
  subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, handleIMUMessage);


  // if (multiScanRegistration->setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  // }

  return 0;
}
