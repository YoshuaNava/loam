
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "loam_velodyne/LaserOdometry.h"
#include "common.h"
#include "loam_msgs/PoseUpdate.h"


std::unique_ptr<loam::LaserOdometry> laserOdometry;

nav_msgs::Odometry laserOdometryMsg;       ///< laser odometry message
tf::StampedTransform laserOdometryTrans;   ///< laser odometry transformation

ros::Publisher pubLaserCloudCornerLast;  ///< last corner cloud message publisher
ros::Publisher pubLaserCloudSurfLast;    ///< last surface cloud message publisher
ros::Publisher pubLaserCloudFullRes;     ///< full resolution cloud message publisher
ros::Publisher pubLaserOdometry;         ///< laser odometry publisher
tf::TransformBroadcaster* tfBroadcaster_ptr;  ///< laser odometry transform broadcaster

ros::Subscriber subCornerPointsSharp;      ///< sharp corner cloud message subscriber
ros::Subscriber subCornerPointsLessSharp;  ///< less sharp corner cloud message subscriber
ros::Subscriber subSurfPointsFlat;         ///< flat surface cloud message subscriber
ros::Subscriber subSurfPointsLessFlat;     ///< less flat surface cloud message subscriber
ros::Subscriber subLaserCloudFullRes;      ///< full resolution cloud message subscriber
ros::Subscriber subImuTrans;               ///< IMU transformation information message subscriber

ros::ServiceServer reset_service;
ros::ServiceServer pose_correction_service;


bool resetCallback(std_srvs::Empty::Request  &req,
                   std_srvs::Empty::Response &res)
{
  ROS_INFO("Laser odometry RESET");
  laserOdometry->correctEstimate();
  return true;
}

bool correctPoseCallback(loam_msgs::PoseUpdate::Request  &req,
                         loam_msgs::PoseUpdate::Response &res)
{
  ROS_ERROR("Laser odometry CORRECTION");
  Eigen::Vector3d pos(req.pose.position.x, req.pose.position.y, req.pose.position.z);
  Eigen::Quaterniond rot(req.pose.orientation.w, req.pose.orientation.z, -req.pose.orientation.x, -req.pose.orientation.y);
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0,1,2);
  res.success = true;

  // loam::Twist transform = laserOdometry->transformSum();
  // ROS_INFO("Transform sum before correction");
  // std::cout << "pos ->  " << transform.pos.x() << ", " << transform.pos.y() << ", " <<  transform.pos.z() << std::endl;
  // std::cout << "rpy ->  " << transform.rot_z.rad() << ", " << -transform.rot_x.rad() << ", " << -transform.rot_y.rad() << std::endl;
  // ROS_INFO("Received transform");
  // std::cout << "pos ->  " << pos.transpose() << std::endl;
  // std::cout << "rpy ->  " << rpy.transpose() << std::endl;

  laserOdometry->correctEstimate(pos, rpy);

  return true;
}

/** \brief Handler method for a new sharp corner cloud.
 *
 * @param cornerPointsSharpMsg the new sharp corner cloud message
 */
void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsSharpMsg)
{
  laserOdometry->timeCornerPointsSharp() = cornerPointsSharpMsg->header.stamp.toSec();
  pcl::fromROSMsg(*cornerPointsSharpMsg, *laserOdometry->cornerPointsSharp());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserOdometry->cornerPointsSharp(), *laserOdometry->cornerPointsSharp(), indices);
  laserOdometry->newCornerPointsSharp() = true;
}


/** \brief Handler method for a new less sharp corner cloud.
 *
 * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
 */
void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLessSharpMsg)
{
  laserOdometry->timeCornerPointsLessSharp() = cornerPointsLessSharpMsg->header.stamp.toSec();
  pcl::fromROSMsg(*cornerPointsLessSharpMsg, *laserOdometry->cornerPointsLessSharp());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserOdometry->cornerPointsLessSharp(), *laserOdometry->cornerPointsLessSharp(), indices);
  laserOdometry->newCornerPointsLessSharp() = true;
}


/** \brief Handler method for a new flat surface cloud.
 *
 * @param surfPointsFlatMsg the new flat surface cloud message
 */
void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsFlatMsg)
{
  laserOdometry->timeSurfPointsFlat() = surfPointsFlatMsg->header.stamp.toSec();
  pcl::fromROSMsg(*surfPointsFlatMsg, *laserOdometry->surfPointsFlat());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserOdometry->surfPointsFlat(), *laserOdometry->surfPointsFlat(), indices);
  laserOdometry->newSurfPointsFlat() = true;
}


/** \brief Handler method for a new less flat surface cloud.
 *
 * @param surfPointsLessFlatMsg the new less flat surface cloud message
 */
void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr& surfPointsLessFlatMsg)
{
  laserOdometry->timeSurfPointsLessFlat() = surfPointsLessFlatMsg->header.stamp.toSec();
  pcl::fromROSMsg(*surfPointsLessFlatMsg, *laserOdometry->surfPointsLessFlat());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*laserOdometry->surfPointsLessFlat(), *laserOdometry->surfPointsLessFlat(), indices);
  laserOdometry->newSurfPointsLessFlat() = true;
}


/** \brief Handler method for a new full resolution cloud.
 *
 * @param laserCloudFullResMsg the new full resolution cloud message
 */
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
  laserOdometry->timeLaserCloudFullRes() = laserCloudFullResMsg->header.stamp.toSec();
  pcl::fromROSMsg(*laserCloudFullResMsg, *laserOdometry->laserCloudFullRes());
  laserOdometry->newLaserCloudFullRes() = true;
}



/** \brief Handler method for a new IMU transformation information.
 *
 * @param laserCloudFullResMsg the new IMU transformation information message
 */
void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTransMsg)
{
  laserOdometry->timeImuTrans() = imuTransMsg->header.stamp.toSec();

  pcl::PointCloud<pcl::PointXYZ> imuTrans;
  pcl::fromROSMsg(*imuTransMsg, imuTrans);

  laserOdometry->imuPitchStart() = imuTrans.points[0].x;
  laserOdometry->imuYawStart() = imuTrans.points[0].y;
  laserOdometry->imuRollStart() = imuTrans.points[0].z;

  laserOdometry->imuPitchEnd() = imuTrans.points[1].x;
  laserOdometry->imuYawEnd() = imuTrans.points[1].y;
  laserOdometry->imuRollEnd() = imuTrans.points[1].z;

  laserOdometry->imuShiftFromStart() = imuTrans.points[2];
  laserOdometry->imuVeloFromStart() = imuTrans.points[3];

  laserOdometry->newImuTrans() = true;
}


/** \brief Publish the current results via the respective topics. */
void publishResults()
{
  loam::Twist transformSum = laserOdometry->transformSum();
  ros::Time sweepTime(laserOdometry->timeSurfPointsLessFlat());

//   // publish odometry tranformations
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformSum.rot_z.rad(),
                                                                              -transformSum.rot_x.rad(),
                                                                              -transformSum.rot_y.rad());

  laserOdometryMsg.header.stamp = sweepTime;
  laserOdometryMsg.pose.pose.orientation.x = -geoQuat.y;
  laserOdometryMsg.pose.pose.orientation.y = -geoQuat.z;
  laserOdometryMsg.pose.pose.orientation.z = geoQuat.x;
  laserOdometryMsg.pose.pose.orientation.w = geoQuat.w;
  laserOdometryMsg.pose.pose.position.x = transformSum.pos.x();
  laserOdometryMsg.pose.pose.position.y = transformSum.pos.y();
  laserOdometryMsg.pose.pose.position.z = transformSum.pos.z();
  pubLaserOdometry.publish(laserOdometryMsg);

  laserOdometryTrans.stamp_ = sweepTime;
  laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  laserOdometryTrans.setOrigin(tf::Vector3( transformSum.pos.x(), transformSum.pos.y(), transformSum.pos.z()) );
  tfBroadcaster_ptr->sendTransform(laserOdometryTrans);


  // publish transformed full resolution input cloud
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr registered_cloud(new pcl::PointCloud<pcl::PointXYZHSV>());
  if(laserOdometry->generateRegisteredCloud(registered_cloud)) {

    loam::publishCloudMsg(pubLaserCloudCornerLast, *laserOdometry->lastCornerCloud(), sweepTime, "/camera");
    loam::publishCloudMsg(pubLaserCloudSurfLast, *laserOdometry->lastSurfaceCloud(), sweepTime, "/camera");
    loam::publishCloudMsg(pubLaserCloudFullRes, *registered_cloud, sweepTime, "/camera");
  }
}


loam::LaserOdometryParams loadParameters(ros::NodeHandle &node,
                    ros::NodeHandle &privateNode)
{
  loam::LaserOdometryParams params = loam::LaserOdometryParams();
  const char* module_name = "LaserOdometry";
  // fetch laser odometry params
  float fParam;
  int iParam;

  if (privateNode.getParam("/loam/scan_period", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid scan_period parameter: %f (expected > 0)", module_name, fParam);
      return false;
    } else {
      params.scanPeriod = fParam;
      // ROS_INFO("%s: Set scan_period: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/odometry/io_ratio", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid io_ratio parameter: %d (expected > 0)", module_name, iParam);
      return false;
    } else {
      params.ioRatio = iParam;
      // ROS_INFO("%s: Set io_ratio: %d", module_name, iParam);
    }
  }

  if (privateNode.getParam("/loam/odometry/max_iterations", iParam)) {
    if (iParam < 1) {
      ROS_ERROR("%s: Invalid max_iterations parameter: %d (expected > 0)", module_name, iParam);
      return false;
    } else {
      params.maxIterations = iParam;
      // ROS_INFO("%s: Set max_iterations: %d", module_name, iParam);
    }
  }

  if (privateNode.getParam("/loam/odometry/delta_T_abort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid delta_T_abort parameter: %f (expected > 0)", module_name, fParam);
      return false;
    } else {
      params.deltaTAbort = fParam;
      // ROS_INFO("%s: Set delta_T_abort: %g", module_name, fParam);
    }
  }

  if (privateNode.getParam("/loam/odometry/delta_R_abort", fParam)) {
    if (fParam <= 0) {
      ROS_ERROR("%s: Invalid delta_R_abort parameter: %f (expected > 0)", module_name, fParam);
      return false;
    } else {
      params.deltaRAbort = fParam;
      // ROS_INFO("%s: Set delta_R_abort: %g", module_name, fParam);
    }
  }

  return params;
}


/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::LaserOdometryParams params = loadParameters(node, privateNode);
  laserOdometry.reset(new loam::LaserOdometry(params));

  // // advertise laser odometry topics
  pubLaserCloudCornerLast = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);
  pubLaserCloudSurfLast   = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);
  pubLaserCloudFullRes    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_odom", 2);
  pubLaserOdometry        = node.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);
  tfBroadcaster_ptr = new tf::TransformBroadcaster();

  // subscribe to scan registration topics
  subCornerPointsSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_sharp", 2, laserCloudSharpHandler);

  subCornerPointsLessSharp = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);

  subSurfPointsFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_flat", 2, laserCloudFlatHandler);

  subSurfPointsLessFlat = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

  subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud", 2, laserCloudFullResHandler);

  // subscribe to IMU topic
  subImuTrans = node.subscribe<sensor_msgs::PointCloud2>("/imu_trans", 5, imuTransHandler);

  // advertise reset and pose correction services
  reset_service = node.advertiseService("laserOdometry/reset", resetCallback);
  pose_correction_service = node.advertiseService("laserOdometry/correct_pose", correctPoseCallback);

    // initialize odometry and odometry tf messages
  laserOdometryMsg.header.frame_id = "/camera_init";
  laserOdometryMsg.child_frame_id = "/laser_odom";
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/laser_odom";

  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();

    // initialization successful
    if(laserOdometry->process())
      publishResults();

    rate.sleep();
  }

  laserOdometry.reset();

  return 0;
}
