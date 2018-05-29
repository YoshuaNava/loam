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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H

#include "loam_utils/common.h" 
#include "loam_utils/Twist.h"
#include "loam_utils/CircularBuffer.h"
#include "loam_utils/IMUState.h"
#include "loam/Parameters.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


namespace loam {

/** \brief Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping {
public:
  explicit LaserMapping(const LaserMappingParams& params = LaserMappingParams());

  /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
  void spin();

  /** \brief Try to process buffered data. */
  bool process();

  bool generateMapCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& map_cloud);

  bool generateRegisteredCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& registered_cloud);


  LaserMappingParams& params() {
    return _params;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudCornerLast() {
    return _laserCloudCornerLast;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudSurfLast() {
    return _laserCloudSurfLast;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudFullRes() {
    return _laserCloudFullRes;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudSurroundDS() {
    return _laserCloudSurroundDS;
  }

  CircularBuffer<IMUState>& imuHistory() {
    return _imuHistory;
  }

  Twist& transformSum() {
    return _transformSum;
  }

  Twist& transformBefMapped() {
    return _transformBefMapped;
  }

  Twist& transformAftMapped() {
    return _transformAftMapped;
  }

  Time& timeLaserCloudCornerLast() {
    return _timeLaserCloudCornerLast;
  }

  Time& timeLaserCloudSurfLast() {
    return _timeLaserCloudSurfLast;
  }

  Time& timeLaserCloudFullRes() {
    return _timeLaserCloudFullRes;
  }

  Time& timeLaserOdometry() {
    return _timeLaserOdometry;
  }

  bool& newLaserCloudCornerLast() {
    return _newLaserCloudCornerLast;
  }

  bool& newLaserCloudSurfLast() {
    return _newLaserCloudSurfLast;
  }

  bool& newLaserCloudFullRes() {
    return _newLaserCloudFullRes;
  }

  bool& newLaserOdometry() {
    return _newLaserOdometry;
  }


protected:
  /** \brief Reset flags, etc. */
  void reset();

  /** \brief Check if all required information for a new processing step is available. */
  bool hasNewData();

  /** Run an optimization. */
  void optimizeTransformTobeMapped();

  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
  void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);


private:

  size_t toIndex(int i, int j, int k) const
  {
    return i + _params.laserCloudWidth * j + _params.laserCloudWidth * _params.laserCloudHeight * k;
  }

  LaserMappingParams _params;

  long _frameCount;
  long _mapFrameCount;

  int _laserCloudCenWidth;
  int _laserCloudCenHeight;
  int _laserCloudCenDepth;

  Time _timeLaserCloudCornerLast;   ///< time of current last corner cloud
  Time _timeLaserCloudSurfLast;     ///< time of current last surface cloud
  Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
  Time _timeLaserOdometry;          ///< time of current laser odometry

  bool _newLaserCloudCornerLast;  ///< flag if a new last corner cloud has been received
  bool _newLaserCloudSurfLast;    ///< flag if a new last surface cloud has been received
  bool _newLaserCloudFullRes;     ///< flag if a new full resolution cloud has been received
  bool _newLaserOdometry;         ///< flag if a new laser odometry has been received

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

  std::vector<size_t> _laserCloudValidInd;
  std::vector<size_t> _laserCloudSurroundInd;

  Twist _transformSum;
  Twist _transformIncre;
  Twist _transformTobeMapped;
  Twist _transformBefMapped;
  Twist _transformAftMapped;

  CircularBuffer<IMUState> _imuHistory;    ///< history of IMU states

  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds
  pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map
};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
