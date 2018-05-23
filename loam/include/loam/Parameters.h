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

#ifndef LOAM_PARAMETERS_H
#define LOAM_PARAMETERS_H


#include "loam_utils/common.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace loam {

struct ScanRegistrationParams {
  /** The time per scan. */
  float scanPeriod;

  /** The size of the IMU history state buffer. */
  int imuHistorySize;

  /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
  int nFeatureRegions;

  /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
  int curvatureRegion;

  /** The maximum number of sharp corner points per feature region. */
  int maxCornerSharp;

  /** The maximum number of less sharp corner points per feature region. */
  int maxCornerLessSharp;

  /** The maximum number of flat surface points per feature region. */
  int maxSurfaceFlat;

  /** The voxel size used for down sizing the remaining less flat surface points. */
  float lessFlatFilterSize;

  /** The curvature threshold below / above a point is considered a flat / corner point. */
  float surfaceCurvatureThreshold;

  /** **/
  int systemDelay;

  /** The model of the multi-beam LiDAR to use **/
  std::string lidarModel;
  
  /** **/
  float vAngleMin, vAngleMax;

  ScanRegistrationParams(const float& scanPeriod_ = 0.1,
                        const int& imuHistorySize_ = 200,
                        const int& nFeatureRegions_ = 6,
                        const int& curvatureRegion_ = 5,
                        const int& maxCornerSharp_ = 2,
                        const int& maxSurfaceFlat_ = 4,
                        const float& lessFlatFilterSize_ = 0.2,
                        const float& surfaceCurvatureThreshold_ = 0.1,
                        const int& systemDelay_ = 20,
                        const std::string lidarModel_ = "none")
  : scanPeriod(scanPeriod_),
    imuHistorySize(imuHistorySize_),
    nFeatureRegions(nFeatureRegions_),
    curvatureRegion(curvatureRegion_),
    maxCornerSharp(maxCornerSharp_),
    maxCornerLessSharp(10 * maxCornerSharp_),
    maxSurfaceFlat(maxSurfaceFlat_),
    lessFlatFilterSize(lessFlatFilterSize_),
    surfaceCurvatureThreshold(surfaceCurvatureThreshold_),
    systemDelay(systemDelay_),
    lidarModel(lidarModel_)
  { }

};




struct LaserOdometryParams {
  float scanPeriod;
  uint16_t ioRatio;       ///< ratio of input to output frames
  size_t maxIterations;   ///< maximum number of iterations
  float deltaTAbort;     ///< optimization abort threshold for deltaT
  float deltaRAbort;     ///< optimization abort threshold for deltaR

  LaserOdometryParams(const float& scanPeriod_ = 0.1,
                      const uint16_t& ioRatio_ = 2,
                      const size_t& maxIterations_ = 25,
                      const float& deltaTAbort_ = 0.1,
                      const float& deltaRAbort_ = 0.1)
  : scanPeriod(scanPeriod_),
    ioRatio(ioRatio_),
    maxIterations(maxIterations_),
    deltaTAbort(deltaTAbort_),
    deltaRAbort(deltaRAbort_)
  { }
};


struct LaserMappingParams {
  float scanPeriod;
  int stackFrameNum;
  int mapFrameNum;
  size_t maxIterations;  ///< maximum number of iterations
  float deltaTAbort;     ///< optimization abort threshold for deltaT
  float deltaRAbort;     ///< optimization abort threshold for deltaR

  float cornerFilterSize;
  float surfFilterSize;
  float mapFilterSize;

  static const size_t laserCloudWidth = 21;
  static const size_t laserCloudHeight = 11;
  static const size_t laserCloudDepth = 21;
  static const size_t laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

  LaserMappingParams(const float& scanPeriod_ = 0.1,
                    const int& stackFrameNum_ = 1,
                    const int& mapFrameNum_ = 5,
                    const size_t& maxIterations_ = 10,
                    const float& deltaTAbort_ = 0.05,
                    const float& deltaRAbort_ = 0.05,
                    const float& cornerFilterSize_ = 0.2,
                    const float& surfFilterSize_ = 0.4,
                    const float& mapFilterSize_ = 0.6)
  : scanPeriod(scanPeriod_),
    stackFrameNum(stackFrameNum_),
    mapFrameNum(mapFrameNum_),
    maxIterations(maxIterations_),
    deltaTAbort(deltaTAbort_),
    deltaRAbort(deltaRAbort_),
    cornerFilterSize(cornerFilterSize_),
    surfFilterSize(surfFilterSize_),
    mapFilterSize(mapFilterSize_)
  { }

};


// struct TransformMaintenanceParams {
//   float scanPeriod;          ///< time per scan

// };

struct LoamParams {
  ScanRegistrationParams scanRegistrationParams;
  LaserOdometryParams laserOdometryParams;
  LaserMappingParams laserMappingParams;
  // TransformMaintenanceParams transformMaintenanceParams;
};

} // end namespace loam

#endif //LOAM_PARAMETERS_H
