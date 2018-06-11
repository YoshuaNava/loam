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

#ifndef LOAM_CT_SCAN_REGISTRATION_H
#define LOAM_CT_SCAN_REGISTRATION_H

#include <ros/ros.h>
#include "loam/ScanRegistration.h"
#include "loam_utils/math_utils.h"

#include <sensor_msgs/PointCloud2.h>


namespace loam {


/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class CtScanRegistration : virtual public ScanRegistration {
public:
  // CtScanRegistration(const RegistrationParams& config = RegistrationParams());

  CtScanRegistration(const ScanRegistrationParams& params = ScanRegistrationParams());


  bool process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
               const Time& scanTime);

  void extractFeatures(const uint16_t& beginIdx = 0);

  /** \brief Setup component in active mode.
   *
   * @param node the ROS node handle
   * @param privateNode the private ROS node handle
   */
  // bool setup(ros::NodeHandle& node,
  //            ros::NodeHandle& privateNode);

  /** \brief Handler method for input cloud messages.
   *
   * @param laserCloudMsg the new input cloud message to process
   */
  // void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);




protected:
  int _systemDelay;             ///< system startup delay counter
  int _laserRotDir;

  ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber


private:
  static const int SYSTEM_DELAY = 20;
};

} // end namespace loam


#endif // LOAM_CT_SCAN_REGISTRATION_H
