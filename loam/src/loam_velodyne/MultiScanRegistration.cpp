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

#include "loam_velodyne/MultiScanRegistration.h"

#include <pcl_conversions/pcl_conversions.h>

#include "loam_utils/math_utils.h"


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const ScanRegistrationParams& params)
    : ScanRegistration(params),
      _systemDelay(SYSTEM_DELAY) { 
  createScanMapper();
}

void MultiScanRegistration::createScanMapper()
{
  _scanMapper = loam::MultiScanMapper();

  // fetch scan matching params
  if (_params.lidarModel != "linear" && _params.lidarModel != "none") {
    if (_params.lidarModel == "VLP-16") {
      _scanMapper = loam::MultiScanMapper::Velodyne_VLP_16();
    } else if (_params.lidarModel == "HDL-32") {
      _scanMapper = loam::MultiScanMapper::Velodyne_HDL_32();
    } else if (_params.lidarModel == "HDL-64E") {
      _scanMapper = loam::MultiScanMapper::Velodyne_HDL_64E();
    }
  } else if (_params.lidarModel == "linear") {
    _scanMapper.set(_params.vAngleMin, _params.vAngleMax, _params.nScanRings);
  } else {
    _scanMapper = loam::MultiScanMapper::Velodyne_VLP_16();
  }
}

bool MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                                    const Time& scanTime)
{
  if (_systemDelay > 0) {
    _systemDelay --;
    return false;
  }

  size_t cloudSize = laserCloudIn.size();

  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZHSV point;
  std::vector<pcl::PointCloud<pcl::PointXYZHSV> > laserCloudScans(_scanMapper.getNumberOfScanRings());

  // extract valid points from input cloud
  for (size_t i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

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

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = _params.scanPeriod * (ori - startOri) / (endOri - startOri);
    point.h = scanID + relTime;

    // project point to the start of the sweep using corresponding IMU data
    if (hasIMUData()) {
      setIMUTransformFor(relTime);
      transformToStartIMU(point);
    }

    laserCloudScans[scanID].push_back(point);
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  for (size_t i = 0; i < _scanMapper.getNumberOfScanRings(); i++) {
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  // extract features
  extractFeatures();

  return true;

  // publish result
  // publishResult();
}

void MultiScanRegistration::extractFeatures(const uint16_t& beginIdx)
{
  // extract features from individual scans
  size_t nScans = _scanIndices.size();
  for (size_t i = beginIdx; i < nScans; i++) {
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZHSV>);
    size_t scanStartIdx = _scanIndices[i].first;
    size_t scanEndIdx = _scanIndices[i].second;

    // skip empty scans
    if (scanEndIdx <= scanStartIdx + 2 * _params.curvatureRegion) {
      continue;
    }

    // reset scan buffers
    extractValidPoints(scanStartIdx, scanEndIdx);

    // extract features from equally sized scan regions
    for (size_t j = 0; j < _params.nFeatureRegions; j++) {
      size_t sp = ((scanStartIdx + _params.curvatureRegion) * (_params.nFeatureRegions - j)
                   + (scanEndIdx - _params.curvatureRegion) * j) / _params.nFeatureRegions;
      size_t ep = ((scanStartIdx + _params.curvatureRegion) * (_params.nFeatureRegions - 1 - j)
                   + (scanEndIdx - _params.curvatureRegion) * (j + 1)) / _params.nFeatureRegions - 1;

      // skip empty regions
      if (ep <= sp) {
        continue;
      }

      size_t regionSize = ep - sp + 1;

      // reset region buffers and estimate curvature
      estimateCurvature(sp, ep);


      // extract corner features
      size_t largestPickedNum = 0;
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
    pcl::PointCloud<pcl::PointXYZHSV> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(_params.lessFlatFilterSize, _params.lessFlatFilterSize, _params.lessFlatFilterSize);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    _surfacePointsLessFlat += surfPointsLessFlatScanDS;
  }
}

} // end namespace loam
