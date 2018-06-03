
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "loam_velodyne/LaserMapping.h"
#include "common.h"
#include "loam_msgs/PoseUpdate.h"


std::unique_ptr<loam::LaserMapping> laserMapping;

nav_msgs::Odometry odomAftMapped;      ///< mapping odometry message
tf::StampedTransform aftMappedTrans;   ///< mapping odometry transformation

ros::Publisher pubLaserCloudSurround;    ///< map cloud message publisher
ros::Publisher pubLaserCloudFullRes;     ///< current full resolution cloud message publisher
ros::Publisher pubOdomAftMapped;         ///< mapping odometry publisher
tf::TransformBroadcaster* tfBroadcaster_ptr;  ///< mapping odometry transform broadcaster

ros::Subscriber subLaserCloudCornerLast;   ///< last corner cloud message subscriber
ros::Subscriber subLaserCloudSurfLast;     ///< last surface cloud message subscriber
ros::Subscriber subLaserCloudFullRes;      ///< full resolution cloud message subscriber
ros::Subscriber subLaserOdometry;          ///< laser odometry message subscriber
ros::Subscriber subImu;                    ///< IMU message subscriber

ros::ServiceServer reset_service;
ros::ServiceServer pose_correction_service;


bool resetCallback(std_srvs::Empty::Request  &req,
                   std_srvs::Empty::Response &res)
{
  ROS_INFO("Laser mapping RESET");
  laserMapping->correctEstimate();
  return true;
}

bool correctPoseCallback(loam_msgs::PoseUpdate::Request  &req,
                         loam_msgs::PoseUpdate::Response &res)
{
  ROS_ERROR("Laser mapping CORRECTION");
  Eigen::Vector3d pos(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  Eigen::Quaterniond rot(req.pose.orientation.w, -req.pose.orientation.y, -req.pose.orientation.z, req.pose.orientation.x);
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0,1,2);
  res.success = true;

  // loam::Twist transform = laserMapping->transformTobeMapped();
  // ROS_INFO("Transform sum before correction");
  // std::cout << "pos ->  " << transform.pos.x() << ", " << transform.pos.y() << ", " <<  transform.pos.z() << std::endl;
  // std::cout << "rpy ->  " << transform.rot_z.rad() << ", " << -transform.rot_x.rad() << ", " << -transform.rot_y.rad() << std::endl;
  // ROS_INFO("Received transform");
  // std::cout << "pos ->  " << pos.transpose() << std::endl;
  // std::cout << "rpy ->  " << rpy.transpose() << std::endl;

  laserMapping->correctEstimate(pos, rpy);
  return true;
}

/** \brief Handler method for a new last corner cloud.
 *
 * @param cornerPointsLastMsg the new last corner cloud message
 */
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg)
{
  laserMapping->timeLaserCloudCornerLast() = cornerPointsLastMsg->header.stamp.toSec();
  pcl::fromROSMsg(*cornerPointsLastMsg, *laserMapping->laserCloudCornerLast());
  laserMapping->newLaserCloudCornerLast() = true;
}


/** \brief Handler method for a new last surface cloud.
 *
 * @param surfacePointsLastMsg the new last surface cloud message
 */
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg)
{
  laserMapping->timeLaserCloudSurfLast() = surfacePointsLastMsg->header.stamp.toSec();
  pcl::fromROSMsg(*surfacePointsLastMsg, *laserMapping->laserCloudSurfLast());
  laserMapping->newLaserCloudSurfLast() = true;
}


/** \brief Handler method for a new full resolution cloud.
 *
 * @param laserCloudFullResMsg the new full resolution cloud message
 */
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
  laserMapping->timeLaserCloudFullRes() = laserCloudFullResMsg->header.stamp.toSec();
  pcl::fromROSMsg(*laserCloudFullResMsg, *laserMapping->laserCloudFullRes());
  laserMapping->newLaserCloudFullRes() = true;
}


/** \brief Handler method for a new laser odometry.
 *
 * @param laserOdometry the new laser odometry message
 */
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  loam::Twist transformSum;
  transformSum.rot_x = -pitch;
  transformSum.rot_y = -yaw;
  transformSum.rot_z = roll;
  transformSum.pos.x() = float(laserOdometry->pose.pose.position.x);
  transformSum.pos.y() = float(laserOdometry->pose.pose.position.y);
  transformSum.pos.z() = float(laserOdometry->pose.pose.position.z);

  laserMapping->timeLaserOdometry() = laserOdometry->header.stamp.toSec();
  laserMapping->transformSum() = transformSum;
  laserMapping->newLaserOdometry() = true;
}


/** \brief Handler method for IMU messages.
 *
 * @param imuIn the new IMU message
 */
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  loam::IMUState newState;
  newState.stamp = imuIn->header.stamp.toSec();
  newState.roll = roll;
  newState.pitch = pitch;

  laserMapping->imuHistory().push(newState);
}


/** \brief Publish the current result via the respective topics. */
void publishResults()
{
  loam::Twist transformAftMapped = laserMapping->transformAftMapped();
  loam::Twist transformBefMapped = laserMapping->transformBefMapped();
  loam::Time timeLaserOdometry = laserMapping->timeLaserOdometry();
  
  // publish new map cloud according to the input output ratio
  if(pubLaserCloudSurround.getNumSubscribers()) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if(laserMapping->generateMapCloud(map_cloud))
      loam::publishCloudMsg(pubLaserCloudSurround, *map_cloud, ros::Time(timeLaserOdometry), "/camera_init");
  }

  // publish transformed full resolution input cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if(laserMapping->generateRegisteredCloud(registered_cloud))
    loam::publishCloudMsg(pubLaserCloudFullRes, *registered_cloud, ros::Time(timeLaserOdometry), "/camera_init");

  // publish odometry after mapped transformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      ( transformAftMapped.rot_z.rad(),
        -transformAftMapped.rot_x.rad(),
        -transformAftMapped.rot_y.rad());

  odomAftMapped.header.stamp = ros::Time(timeLaserOdometry);
  odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
  odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
  odomAftMapped.pose.pose.orientation.z = geoQuat.x;
  odomAftMapped.pose.pose.orientation.w = geoQuat.w;
  odomAftMapped.pose.pose.position.x = transformAftMapped.pos.x();
  odomAftMapped.pose.pose.position.y = transformAftMapped.pos.y();
  odomAftMapped.pose.pose.position.z = transformAftMapped.pos.z();
  odomAftMapped.twist.twist.angular.x = transformBefMapped.rot_x.rad();
  odomAftMapped.twist.twist.angular.y = transformBefMapped.rot_y.rad();
  odomAftMapped.twist.twist.angular.z = transformBefMapped.rot_z.rad();
  odomAftMapped.twist.twist.linear.x = transformBefMapped.pos.x();
  odomAftMapped.twist.twist.linear.y = transformBefMapped.pos.y();
  odomAftMapped.twist.twist.linear.z = transformBefMapped.pos.z();
  pubOdomAftMapped.publish(odomAftMapped);

  aftMappedTrans.stamp_ = ros::Time(timeLaserOdometry);
  aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped.pos.x(),
                                        transformAftMapped.pos.y(),
                                        transformAftMapped.pos.z()));
  tfBroadcaster_ptr->sendTransform(aftMappedTrans);
}


loam::LaserMappingParams loadParameters(ros::NodeHandle& node,
                                        ros::NodeHandle& privateNode)
{
  loam::LaserMappingParams params = loam::LaserMappingParams();
  const char* module_name = "LaserMapping:";
  // fetch laser mapping params
  float fParam;
  int iParam;

  if (privateNode.getParam("/loam/scan_period", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid scan_period parameter: %f (expected > 0)", module_name, fParam);
    } else {
      params.scanPeriod = fParam;
      // ROS_INFO("%s: Set scan_period: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/max_iterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_iterations parameter: %d (expected > 0)", module_name, iParam);
    } else {
      params.maxIterations = iParam;
      // ROS_INFO("%s: Set max_iterations: %d", module_name, iParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/delta_T_abort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid delta_T_abort parameter: %f (expected > 0)", module_name, fParam);
    } else {
      params.deltaTAbort = fParam;
      // ROS_INFO("%s: Set delta_T_abort: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/delta_R_abort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid delta_R_abort parameter: %f (expected > 0)", module_name, fParam);
    } else {
      params.deltaRAbort = fParam;
      // ROS_INFO("%s: Set delta_R_abort: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/corner_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid cornerFilterSize parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.cornerFilterSize = fParam;
      // ROS_INFO("%s: Set corner down size filter leaf size: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/surface_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid surface_filter_size parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.surfFilterSize = fParam;
      // ROS_INFO("%s: Set surface down size filter leaf size: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/mapping/map_filter_size", fParam)) {
    if (fParam < 0.001) {
      ROS_ERROR("%s: Invalid map_filter_size parameter: %f (expected >= 0.001)", module_name, fParam);
    } else {
      params.mapFilterSize = fParam;
      // ROS_INFO("%s: Set map down size filter leaf size: %g", module_name, fParam);
    }
  }

  return params;
}



/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserMappingParams params = loadParameters(node, privateNode);
  laserMapping.reset(new loam::LaserMapping(params));

  // advertise laser mapping topics
  pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 1);
  pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_registered", 2);
  pubOdomAftMapped = node.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
  tfBroadcaster_ptr = new tf::TransformBroadcaster();

  // subscribe to laser odometry topics
  subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, laserCloudCornerLastHandler);

  subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, laserCloudSurfLastHandler);

  subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, laserOdometryHandler);

  subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_odom", 2, laserCloudFullResHandler);

  // subscribe to IMU topic
  subImu = node.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  // advertise reset and pose correction services
  reset_service = node.advertiseService("laserMapping/reset", resetCallback);
  pose_correction_service = node.advertiseService("laserMapping/correct_pose", correctPoseCallback);

  // initialize mapping odometry and odometry tf messages  
  odomAftMapped.header.frame_id = "/camera_init";
  odomAftMapped.child_frame_id = "/aft_mapped";
  aftMappedTrans.frame_id_ = "/camera_init";
  aftMappedTrans.child_frame_id_ = "/aft_mapped";


  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    // initialization successful
    if(laserMapping->process())
      publishResults();

    rate.sleep();
  }

  laserMapping.reset();

  return 0;
}
