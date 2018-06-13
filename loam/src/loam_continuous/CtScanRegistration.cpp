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

#include "loam_continuous/CtScanRegistration.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


namespace loam {


CtScanRegistration::CtScanRegistration(const ScanRegistrationParams& params)
    : ScanRegistration(params),
      _laserRotDir(1) { 
  _imuTrans = pcl::PointCloud<pcl::PointXYZ>(8, 1);
};

void CtScanRegistration::reset(const Time& scanTime,
                             const bool& newSweep)
{
  _scanTime = scanTime;

  // re-initialize IMU start index and state
  _imuIdx = 0;
  if (hasIMUData()) {
    interpolateIMUStateFor(0, _imuStart);
  }

  // clear internal cloud buffers at the beginning of a sweep
  if (newSweep) {
    _sweepStart = scanTime;

    _imuVelocityShiftLast = _imuCur.velocity - _imuStart.velocity;
    _imuStartLast = _imuStart;
    _imuCurLast = _imuCur;
    _imuPositionShiftLast = _imuPositionShift;

    _imuStart = _imuCur;

    // clear cloud buffers
    _laserCloud.clear();
    _cornerPointsSharp.clear();
    _cornerPointsLessSharp.clear();
    _surfacePointsFlat.clear();
    _surfacePointsLessFlat.clear();

    // clear scan indices vector
    _scanIndices.clear();
  }
}


bool CtScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                                 const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // original loam_continous
  pcl::PointXYZ laserPointFirst = laserCloudIn.points[0];
  pcl::PointXYZ laserPointLast = laserCloudIn.points[cloudSize - 1];

  float rangeFirst = calcPointDistance(laserPointFirst);
  laserPointFirst.x /= rangeFirst;
  laserPointFirst.y /= rangeFirst;
  laserPointFirst.z /= rangeFirst;

  float rangeLast = calcPointDistance(laserPointLast);
  laserPointLast.x /= rangeLast;
  laserPointLast.y /= rangeLast;
  laserPointLast.z /= rangeLast;

  float laserAngle = atan2(laserPointLast.x - laserPointFirst.x,
                           laserPointLast.y - laserPointFirst.y);

  bool newSweep = false;
  if (laserAngle * _laserRotDir < 0 &&
    scanTime - _sweepStart > _params.scanPeriod) {
    // std::cout << "######################    NEW SWEEP!    ######################" << std::endl;
    _laserRotDir *= -1;
    newSweep = true;
  }

  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime, newSweep);

  pcl::PointXYZHSV point;
  pcl::PointCloud<pcl::PointXYZHSV> laserCloudScan;

  // extract valid points from input cloud
  for (size_t i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;
    point.h = scanTime - _sweepStart;
    point.v = 0;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate relative scan time based on point orientation
    float relTime = 0;

    // project point to the start of the sweep using corresponding IMU data
    if (hasIMUData()) {
      setIMUTransformFor(relTime);
      transformToStartIMU(point);
    }

    laserCloudScan.push_back(point);
  }

  _laserCloud = laserCloudScan;

  // extract features
  extractFeatures();

  return true;

  // publish result
  // publishResult();
}


void CtScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  size_t cloudSize = _laserCloud.size();
  size_t startPoints[4] = {5,
                           6 + size_t((cloudSize - 10) / 4.0),
                           6 + size_t((cloudSize - 10) / 2.0),
                           6 + size_t(3 * (cloudSize - 10) / 4.0)};
  size_t endPoints[4] = {5 + size_t((cloudSize - 10) / 4.0),
                         5 + size_t((cloudSize - 10) / 2.0),
                         5 + size_t(3 * (cloudSize - 10) / 4.0),
                         cloudSize - 6};

  // extract features from individual scans
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZHSV>);

  // reset scan buffers. Exclude invalid points
  extractValidPoints(0, cloudSize);

  // extract features from equally sized scan regions
  for (size_t i = 0; i < _params.nFeatureRegions; i++) {
    size_t sp = startPoints[i];
    size_t ep = endPoints[i];
    size_t regionSize = ep - sp;

    // reset region buffers
    estimateCurvature(sp, ep-1);

    // extract corner features
    size_t largestPickedNum = 0;
    for (size_t j = regionSize-1; j > 0 && largestPickedNum < _params.maxCornerLessSharp; j--) {
      size_t scanIdx = _regionSortIndices[j];
      size_t regionIdx = scanIdx - sp;

      if (_scanNeighborPicked[scanIdx] == 0 &&
          _regionCurvature[regionIdx] > _params.surfaceCurvatureThreshold) {

        largestPickedNum++;
        if (largestPickedNum <= _params.maxCornerSharp) {
          _regionLabel[regionIdx] = CORNER_SHARP;
          _cornerPointsSharp.push_back(_laserCloud[scanIdx]);
        } else if (largestPickedNum <= _params.maxCornerSharp*10) {
          _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          _cornerPointsLessSharp.push_back(_laserCloud[scanIdx]);
        }
        else {
          break;
        }
        
        markAsPicked(scanIdx, scanIdx);
      }
    }

    // extract flat surface features
    size_t smallestPickedNum = 0;
    for (size_t j = 0; j < regionSize && smallestPickedNum < _params.maxSurfaceFlat; j++) {
      size_t scanIdx = _regionSortIndices[j];
      size_t regionIdx = scanIdx - sp;

      if (_scanNeighborPicked[scanIdx] == 0 &&
          _regionCurvature[regionIdx] < _params.surfaceCurvatureThreshold) {

        smallestPickedNum++;
        _regionLabel[regionIdx] = SURFACE_FLAT;
        _surfacePointsFlat.push_back(_laserCloud[scanIdx]);

        markAsPicked(scanIdx, scanIdx);
      }
    }

    // extract less flat surface features
    for (size_t j = 0; j < regionSize; j++) {
      if (_regionLabel[j] <= SURFACE_LESS_FLAT) {
        surfPointsLessFlatScan->push_back(_laserCloud[sp + j]);
      }
    }
  }

  // down size less flat surface point cloud of current scan
  pcl::PointCloud<pcl::PointXYZHSV> surfPointsLessFlatScanDS;
  pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
  downSizeFilter.setInputCloud(surfPointsLessFlatScan);
  downSizeFilter.setLeafSize(_params.lessFlatFilterSize, _params.lessFlatFilterSize, _params.lessFlatFilterSize);
  downSizeFilter.filter(surfPointsLessFlatScanDS);

  _surfacePointsLessFlat += surfPointsLessFlatScanDS;

  prepareImuStateMessage(_imuStart, _imuStartLast, _imuCur, _imuCurLast,
                         _imuPositionShift, _imuPositionShiftLast, _imuVelocityShift, _imuVelocityShiftLast);
}

void CtScanRegistration::prepareImuStateMessage(const IMUState imuStart, const IMUState imuStartLast,
                                                const IMUState imuCur, const IMUState imuCurLast,
                                                Vector3 imuPositionShift, Vector3 imuPositionShiftLast,
                                                Vector3 imuVelocityShift, Vector3 imuVelocityShiftLast) 
{
  rotateYXZ(imuPositionShift, -imuStart.yaw, -imuStart.pitch, -imuStart.roll);
  rotateYXZ(imuVelocityShift, -imuStart.yaw, -imuStart.pitch, -imuStart.roll);
  rotateYXZ(imuPositionShiftLast, -imuStartLast.yaw, -imuStartLast.pitch, -imuStartLast.roll);
  rotateYXZ(imuVelocityShiftLast, -imuStartLast.yaw, -imuStartLast.pitch, -imuStartLast.roll);

  _imuTrans.points[0].x = imuStart.pitch.rad();
  _imuTrans.points[0].y = imuStart.yaw.rad();
  _imuTrans.points[0].z = imuStart.roll.rad();

  _imuTrans.points[1].x = imuCur.pitch.rad();
  _imuTrans.points[1].y = imuCur.yaw.rad();
  _imuTrans.points[1].z = imuCur.roll.rad();

  _imuTrans.points[2].x = imuPositionShift.x();
  _imuTrans.points[2].y = imuPositionShift.y();
  _imuTrans.points[2].z = imuPositionShift.z();

  _imuTrans.points[3].x = imuVelocityShift.x();
  _imuTrans.points[3].y = imuVelocityShift.y();
  _imuTrans.points[3].z = imuVelocityShift.z();

  _imuTrans.points[4].x = imuStartLast.pitch.rad();
  _imuTrans.points[4].y = imuStartLast.yaw.rad();
  _imuTrans.points[4].z = imuStartLast.roll.rad();

  _imuTrans.points[5].x = imuCurLast.pitch.rad();
  _imuTrans.points[5].y = imuCurLast.yaw.rad();
  _imuTrans.points[5].z = imuCurLast.roll.rad();

  _imuTrans.points[6].x = imuPositionShiftLast.x();
  _imuTrans.points[6].y = imuPositionShiftLast.y();
  _imuTrans.points[6].z = imuPositionShiftLast.z();

  _imuTrans.points[7].x = imuVelocityShiftLast.x();
  _imuTrans.points[7].y = imuVelocityShiftLast.y();
  _imuTrans.points[7].z = imuVelocityShiftLast.z();
}


} // end namespace loam
