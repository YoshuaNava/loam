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

#include "loam_velodyne/TransformMaintenance.h"


namespace loam {

using std::sin;
using std::cos;
using std::asin;
using std::atan2;


TransformMaintenance::TransformMaintenance()
{
  for (int i = 0; i < 6; i++) {
    _transformSum[i] = 0;
    _transformIncre[i] = 0;
    _transformMapped[i] = 0;
    _transformBefMapped[i] = 0;
    _transformAftMapped[i] = 0;
  }
}

void TransformMaintenance::correctEstimate(const Eigen::Vector3d& pos, 
                                           const Eigen::Vector3d& rpy) {
  std::lock_guard<std::mutex> lock(main_thread_mutex_);

  for (int i = 0; i < 3; i++) {
    _transformSum[3+i] = pos(i);
    _transformAftMapped[3+i] = pos(i);
    _transformMapped[3+i] = pos(i);
    _transformIncre[3+i] = pos(i);
  }
  _transformIncre[0] = -rpy(1);
  _transformIncre[1] = -rpy(2);
  _transformIncre[2] = rpy(0);
  _transformMapped[0] = -rpy(1);
  _transformMapped[1] = -rpy(2);
  _transformMapped[2] = rpy(0);
  _transformAftMapped[0] = -rpy(1);
  _transformAftMapped[1] = -rpy(2);
  _transformAftMapped[2] = rpy(0);
  _transformSum[0] = -rpy(1);
  _transformSum[1] = -rpy(2);
  _transformSum[2] = rpy(0);

  // for (int i = 0; i < 6; i++) {
  //   _transformBefMapped[i] = 0;
  //   std::cout << "loop = " << i << std::endl;
  //   std::cout << _transformAftMapped[i] << std::endl;
  //   std::cout << _transformMapped[i] << std::endl;
  //   std::cout << _transformIncre[i] << std::endl;
  //   std::cout << _transformSum[i] << std::endl;
  // }
}

float* TransformMaintenance::getIntegratedTransform()
{
  return _transformMapped;
}

void TransformMaintenance::transformAssociateToMap()
{
  std::lock_guard<std::mutex> lock(main_thread_mutex_);
  float x1 = cos(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
             - sin(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);
  float y1 = _transformBefMapped[4] - _transformSum[4];
  float z1 = sin(_transformSum[1]) * (_transformBefMapped[3] - _transformSum[3])
             + cos(_transformSum[1]) * (_transformBefMapped[5] - _transformSum[5]);

  float x2 = x1;
  float y2 = cos(_transformSum[0]) * y1 + sin(_transformSum[0]) * z1;
  float z2 = -sin(_transformSum[0]) * y1 + cos(_transformSum[0]) * z1;

  _transformIncre[3] = cos(_transformSum[2]) * x2 + sin(_transformSum[2]) * y2;
  _transformIncre[4] = -sin(_transformSum[2]) * x2 + cos(_transformSum[2]) * y2;
  _transformIncre[5] = z2;

  float sbcx = sin(_transformSum[0]);
  float cbcx = cos(_transformSum[0]);
  float sbcy = sin(_transformSum[1]);
  float cbcy = cos(_transformSum[1]);
  float sbcz = sin(_transformSum[2]);
  float cbcz = cos(_transformSum[2]);

  float sblx = sin(_transformBefMapped[0]);
  float cblx = cos(_transformBefMapped[0]);
  float sbly = sin(_transformBefMapped[1]);
  float cbly = cos(_transformBefMapped[1]);
  float sblz = sin(_transformBefMapped[2]);
  float cblz = cos(_transformBefMapped[2]);

  float salx = sin(_transformAftMapped[0]);
  float calx = cos(_transformAftMapped[0]);
  float saly = sin(_transformAftMapped[1]);
  float caly = cos(_transformAftMapped[1]);
  float salz = sin(_transformAftMapped[2]);
  float calz = cos(_transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
              - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                           - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                           + cblx*salx*sbly)
              - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                           - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                           + cblx*cbly*salx);
  _transformMapped[0] = -asin(srx);

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                       - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
                 - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx)
                              - calx*cblx*cbly*saly)
                 + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                              + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                              + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                       - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
                 + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                              + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                              + calx*caly*cblx*cbly)
                 - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                              + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz)
                              - calx*caly*cblx*sbly);
  _transformMapped[1] = atan2(srycrx / cos(_transformMapped[0]),
                             crycrx / cos(_transformMapped[0]));

  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                               - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                               + cblx*cbly*salx)
                 - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                                 + cblx*salx*sbly)
                 + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                               - calx*salz*(cbly*cblz + sblx*sbly*sblz)
                                               + cblx*salx*sbly)
                 - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*calz*(sbly*sblz + cbly*cblz*sblx)
                                                 + cblx*cbly*salx)
                 + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  _transformMapped[2] = atan2(srzcrx / cos(_transformMapped[0]),
                             crzcrx / cos(_transformMapped[0]));

  x1 = cos(_transformMapped[2]) * _transformIncre[3] - sin(_transformMapped[2]) * _transformIncre[4];
  y1 = sin(_transformMapped[2]) * _transformIncre[3] + cos(_transformMapped[2]) * _transformIncre[4];
  z1 = _transformIncre[5];

  x2 = x1;
  y2 = cos(_transformMapped[0]) * y1 - sin(_transformMapped[0]) * z1;
  z2 = sin(_transformMapped[0]) * y1 + cos(_transformMapped[0]) * z1;

  _transformMapped[3] = _transformAftMapped[3]
                       - (cos(_transformMapped[1]) * x2 + sin(_transformMapped[1]) * z2);
  _transformMapped[4] = _transformAftMapped[4] - y2;
  _transformMapped[5] = _transformAftMapped[5]
                       - (-sin(_transformMapped[1]) * x2 + cos(_transformMapped[1]) * z2);
}



void TransformMaintenance::processOdometryTransform(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
{
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0, 1, 2);
  _transformSum[0] = -rpy(1);
  _transformSum[1] = -rpy(2);
  _transformSum[2] = rpy(0);

  _transformSum[3] = pos(0);
  _transformSum[4] = pos(1);
  _transformSum[5] = pos(2);

  transformAssociateToMap();
}


void TransformMaintenance::processMappingTransform(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot, const Eigen::Vector3d& linear_vel, const Eigen::Vector3d& angular_vel)
{
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0, 1, 2);
  _transformAftMapped[0] = -rpy(1);
  _transformAftMapped[1] = -rpy(2);
  _transformAftMapped[2] = rpy(0);

  _transformAftMapped[3] = pos(0);
  _transformAftMapped[4] = pos(1);
  _transformAftMapped[5] = pos(2);

  _transformBefMapped[0] = angular_vel(0);
  _transformBefMapped[1] = angular_vel(1);
  _transformBefMapped[2] = angular_vel(2);

  _transformBefMapped[3] = linear_vel(0);
  _transformBefMapped[4] = linear_vel(1);
  _transformBefMapped[5] = linear_vel(2);
}

} // end namespace loam
