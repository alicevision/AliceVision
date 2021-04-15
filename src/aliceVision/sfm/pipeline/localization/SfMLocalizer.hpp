// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>

#include <cstddef>
#include <limits>

namespace aliceVision {
namespace sfm {

struct ImageLocalizerMatchData
{
  /// 3x4 matrix represented the estimated camera pose.
  Mat34 projection_matrix; 
  
  /// 3xN matrix storing all the 3D points whose images have been found
  /// in the query view through the feature matching procedure.
  Mat pt3D; 
  
  /// 2xN matrix storing all 2D distorted points associated to 3D points (pt3D)
  /// found through the feature matching procedure.
  Mat pt2D;
  
  /// pt2D and pt3D have the same number of columns.
  /// Index mask for both pt3D and pt2D whose elements
  /// represent the column indices of inliers in  pt2D 
  /// and pt3D.
  std::vector<std::size_t> vec_inliers;

  std::vector<feature::EImageDescriberType> vec_descType;
  
  /// Upper bound pixel(s) tolerance for residual errors
  double error_max = std::numeric_limits<double>::infinity();
  size_t max_iteration = 4096;
};

class SfMLocalizer
{
public:
  virtual ~SfMLocalizer() = default;

  /**
  * @brief Build the retrieval database (3D points descriptors)
  *
  * @param[in] sfmData the SfM scene that have to be described
  * @param[in] regionPerView regions provider
  * @return True if the database has been correctly setup
  */
  virtual bool Init(const sfmData::SfMData& sfmData, const feature::RegionsPerView& regionPerView) = 0;

  /**
  * @brief Try to localize an image in the database
  *
  * @param[in] imageSize the w,h image size
  * @param[in] optionalIntrinsics camera intrinsic if known (else nullptr)
  * @param[in] queryRegions the image regions (type must be the same as the database)
  * @param[in] randomNumberGenerator random number generator
  * @param[out] pose found pose
  * @param[out] resectionData matching data (2D-3D and inliers; optional)
  * @return True if a putative pose has been estimated
  */
  virtual bool Localize(const Pair& imageSize,
                        const camera::IntrinsicBase* optionalIntrinsics,
                        const feature::Regions& queryRegions,
                        std::mt19937 &randomNumberGenerator,
                        geometry::Pose3& pose,
                        ImageLocalizerMatchData* resectionData = nullptr // optional
                        ) const = 0;


  /**
  * @brief Try to localize an image from known 2D-3D matches
  *
  * @param[in] imageSize the w,h image size
  * @param[in] optionalIntrinsics camera intrinsic if known (else nullptr)
  * @param[in,out] resectionData matching data (with filled 2D-3D correspondences).
   * The 2D points are supposed to be the original distorted image points
  * @param[out] pose found pose
  * @param[in] estimator The type of robust estimator to use. The only supported 
   * frameworks are ERobustEstimator::ACRANSAC and ERobustEstimator::LORANSAC.
  * @return True if a putative pose has been estimated
  */
  static bool Localize(const Pair& imageSize,
                       const camera::IntrinsicBase* optionalIntrinsics,
                       std::mt19937 &randomNumberGenerator,
                       ImageLocalizerMatchData& resectionData,
                       geometry::Pose3& pose,
                       robustEstimation::ERobustEstimator estimator = robustEstimation::ERobustEstimator::ACRANSAC);

  /**
  * @brief Refine a pose according 2D-3D matching & camera model data
  *
  * @param[in,out] intrinsics Camera model
  * @param[in,out] pose Camera pose
  * @param[in] matchingData Corresponding 2D-3D data
  * @param[in] refinePose tell if pose must be refined
  * @param[in] refineIntrinsic tell if intrinsics must be refined
  * @return True if the refinement decreased the RMSE pixel residual error
  */
  static bool RefinePose(camera::IntrinsicBase* intrinsics,
                         geometry::Pose3& pose,
                         const ImageLocalizerMatchData& matchingData,
                         bool refinePose,
                         bool refineIntrinsic);
};


} // namespace sfm
} // namespace aliceVision
