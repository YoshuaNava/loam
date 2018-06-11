#ifndef LOAM_SCANREGISTRATION_ROS_H
#define LOAM_SCANREGISTRATION_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include "loam_utils/math_utils.h"
#include "loam_velodyne/MultiScanRegistration.h"
#include "loam_continuous/CtScanRegistration.h"
#include "common.h"


namespace loam {

class ScanRegistrationRos {
public:

  ScanRegistrationRos() {};

  bool setup(ros::NodeHandle& node,
             ros::NodeHandle& privateNode);

  loam::ScanRegistrationParams loadParameters(ros::NodeHandle& node,
                                              ros::NodeHandle& privateNode);


  /** \brief Publish the current results via the respective topics. */
  void publishResults();


  /** \brief Handler method for IMU messages.
   *
   * @param imuIn the new IMU message
   */
  void handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn);


  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudMsg the new input cloud message to process
   */
  void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

protected:

  std::unique_ptr<loam::ScanRegistration> _scanRegistration;

  ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber
  ros::Subscriber _subImu;    ///< IMU message subscriber

  ros::Publisher _pubLaserCloud;              ///< full resolution cloud message publisher
  ros::Publisher _pubCornerPointsSharp;       ///< sharp corner cloud message publisher
  ros::Publisher _pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher
  ros::Publisher _pubSurfPointsFlat;          ///< flat surface cloud message publisher
  ros::Publisher _pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher
  ros::Publisher _pubImuTrans;                ///< IMU transformation message publisher

};

}
#endif //LOAM_MULTISCANREGISTRATION_ROS_H