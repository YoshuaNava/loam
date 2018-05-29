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


namespace loam {

ScanRegistration::ScanRegistration(const ScanRegistrationParams& params)
      : _params(params),
        _sweepStart(0),
        _scanTime(0),
        _imuStart(),
        _imuCur(),
        _imuIdx(0),
        _imuHistory(_params.imuHistorySize),
        _laserCloud(),
        _cornerPointsSharp(),
        _cornerPointsLessSharp(),
        _surfacePointsFlat(),
        _surfacePointsLessFlat(),
        _imuTrans(4, 1),
        _regionCurvature(),
        _regionLabel(),
        _regionSortIndices(),
        _scanNeighborPicked()
{
  _imuHistory.ensureCapacity(params.imuHistorySize);
}


void ScanRegistration::addImuData(IMUState newState)
{
  if (_imuHistory.size() > 0) {
    // accumulate IMU position and velocity over time
    rotateZXY(newState.acceleration, newState.roll, newState.pitch, newState.yaw);

    const IMUState& prevState = _imuHistory.last();
    float timeDiff = float(newState.stamp - prevState.stamp);
    newState.position = prevState.position
                        + (prevState.velocity * timeDiff)
                        + (0.5 * newState.acceleration * timeDiff * timeDiff);
    newState.velocity = prevState.velocity
                        + newState.acceleration * timeDiff;
  }

  _imuHistory.push(newState);
}


void ScanRegistration::reset(const Time& scanTime,
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



void ScanRegistration::setIMUTransformFor(const float& relTime)
{
  interpolateIMUStateFor(relTime, _imuCur);

  float relSweepTime = (_scanTime - _sweepStart) + relTime;
  _imuPositionShift = _imuCur.position - _imuStart.position - _imuStart.velocity * relSweepTime;
}



void ScanRegistration::transformToStartIMU(pcl::PointXYZI& point)
{
  // rotate point to global IMU system
  rotateZXY(point, _imuCur.roll, _imuCur.pitch, _imuCur.yaw);

  // add global IMU position shift
  point.x += _imuPositionShift.x();
  point.y += _imuPositionShift.y();
  point.z += _imuPositionShift.z();

  // rotate point back to local IMU system relative to the start IMU state
  rotateYXZ(point, -_imuStart.yaw, -_imuStart.pitch, -_imuStart.roll);
}



void ScanRegistration::interpolateIMUStateFor(const float &relTime,
                                              IMUState &outputState)
{
  double timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  while (_imuIdx < _imuHistory.size() - 1 && timeDiff > 0) {
    _imuIdx++;
    timeDiff = (_scanTime - _imuHistory[_imuIdx].stamp) + relTime;
  }

  if (_imuIdx == 0 || timeDiff > 0) {
    outputState = _imuHistory[_imuIdx];
  } else {
    float ratio = -timeDiff / (_imuHistory[_imuIdx].stamp - _imuHistory[_imuIdx - 1].stamp);
    IMUState::interpolate(_imuHistory[_imuIdx], _imuHistory[_imuIdx - 1], ratio, outputState);
  }
}



void ScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  size_t nScans = _scanIndices.size();
  for (size_t i = beginIdx; i < nScans; i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * _params.curvatureRegion) {
      continue;
    }

    // reset scan buffers
    setScanBuffersFor(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    for (int j = 0; j < _params.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _params.curvatureRegion) * (_params.nFeatureRegions - j)
                   + (scanEndIdx - _params.curvatureRegion) * j) / _params.nFeatureRegions;
      size_t ep = ((scanStartIdx + _params.curvatureRegion) * (_params.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _params.curvatureRegion) * (j + 1)) / _params.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers
      setRegionBuffersFor(sp, ep);


      // extract corner features
      int largestPickedNum = 0;
      for (size_t k = regionSize; k > 0 && largestPickedNum < _params.maxCornerLessSharp;) {
        size_t idx = _regionSortIndices[--k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] > _params.surfaceCurvatureThreshold) {

          largestPickedNum++;
          if (largestPickedNum <= _params.maxCornerSharp) {
            _regionLabel[regionIdx] = CORNER_SHARP;
            _cornerPointsSharp.push_back(_laserCloud[idx]);
          } else {
            _regionLabel[regionIdx] = CORNER_LESS_SHARP;
          }
          _cornerPointsLessSharp.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract flat surface features
      size_t smallestPickedNum = 0;
      for (size_t k = 0; k < regionSize && smallestPickedNum < (size_t)_params.maxSurfaceFlat; k++) {
        size_t idx = _regionSortIndices[k];
        size_t scanIdx = idx - scanStartIdx;
        size_t regionIdx = idx - sp;

        if (_scanNeighborPicked[scanIdx] == 0 &&
            _regionCurvature[regionIdx] < _params.surfaceCurvatureThreshold) {

          smallestPickedNum++;
          _regionLabel[regionIdx] = SURFACE_FLAT;
          _surfacePointsFlat.push_back(_laserCloud[idx]);

          markAsPicked(idx, scanIdx);
        }
      }

      // extract less flat surface features
      for (size_t k = 0; k < regionSize; k++) {
        if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
          surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
        }
      }
    }

    // down size less flat surface point cloud of current scan
    pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_params.lessFlatFilterSize, _params.lessFlatFilterSize, _params.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
}



void ScanRegistration::setRegionBuffersFor(const size_t& startIdx,
                                           const size_t& endIdx)
{
  // resize buffers
  size_t regionSize = endIdx - startIdx + 1;
  _regionCurvature.resize(regionSize);
  _regionSortIndices.resize(regionSize);
  _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

  // calculate point curvatures and reset sort indices
  float pointWeight = -2 * _params.curvatureRegion;

  for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;

    for (int j = 1; j <= _params.curvatureRegion; j++) {
      diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
      diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
      diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }

    _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    _regionSortIndices[regionIdx] = i;
  }

  // sort point curvatures
  for (size_t i = 1; i < regionSize; i++) {
    for (size_t j = i; j >= 1; j--) {
      if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
        std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
      }
    }
  }
}



void ScanRegistration::setScanBuffersFor(const size_t& startIdx,
                                         const size_t& endIdx)
{
  // resize buffers
  size_t scanSize = endIdx - startIdx + 1;
  _scanNeighborPicked.assign(scanSize, 0);

  // mark unreliable points as picked
  for (size_t i = startIdx + _params.curvatureRegion; i < endIdx - _params.curvatureRegion; i++) {
    const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);
    const pcl::PointXYZI& point = (_laserCloud[i]);
    const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);

    float diffNext = calcSquaredDiff(nextPoint, point);

    if (diffNext > 0.1) {
      float depth1 = calcPointDistance(point);
      float depth2 = calcPointDistance(nextPoint);

      if (depth1 > depth2) {
        float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx - _params.curvatureRegion], _params.curvatureRegion + 1, 1);

          continue;
        }
      } else {
        float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

        if (weighted_distance < 0.1) {
          std::fill_n(&_scanNeighborPicked[i - startIdx + 1], _params.curvatureRegion + 1, 1);
        }
      }
    }

    float diffPrevious = calcSquaredDiff(point, previousPoint);
    float dis = calcSquaredPointDistance(point);

    if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
      _scanNeighborPicked[i - startIdx] = 1;
    }
  }
}



void ScanRegistration::markAsPicked(const size_t& cloudIdx,
                                    const size_t& scanIdx)
{
  _scanNeighborPicked[scanIdx] = 1;

  for (int i = 1; i <= _params.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx + i] = 1;
  }

  for (int i = 1; i <= _params.curvatureRegion; i++) {
    if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
      break;
    }

    _scanNeighborPicked[scanIdx - i] = 1;
  }
}




} // end namespace loam
