
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/numeric/numeric.h"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
#include <openMVG/robust_estimation/robust_estimators.hpp>

namespace openMVG {
namespace sfm {

struct Image_Localizer_Match_Data
{
  Mat34 projection_matrix; // 3x4 matrix represented the estimated camera pose.
  
  Mat pt3D; // 3xN matrix storing all the 3D points whose images have been found
            // in the query view through the feature matching procedure.
  
  Mat pt2D; // 2xN matrix storing all 2D distorted points associated to 3D points (pt3D)
            // found through the feature matching procedure.
  
  // pt2D and pt3D have the same number of columns.
  std::vector<size_t> vec_inliers; // Index mask for both pt3D and pt2D whose elements
                                   // represent the column indices of inliers in  pt2D 
                                   // and pt3D.
  
  // Upper bound pixel(s) tolerance for residual errors
  double error_max = std::numeric_limits<double>::infinity();
  size_t max_iteration = 4096;
};

class SfM_Localizer
{
public:
  virtual ~SfM_Localizer() {}

  /**
  * @brief Build the retrieval database (3D points descriptors)
  *
  * @param[in] sfm_data the SfM scene that have to be described
  * @param[in] region_provider regions provider
  * @return True if the database has been correctly setup
  */
  virtual bool Init
  (
    const SfM_Data & sfm_data,
    const Regions_Provider & region_provider
  ) = 0;

  /**
  * @brief Try to localize an image in the database
  *
  * @param[in] image_size the w,h image size
  * @param[in] optional_intrinsics camera intrinsic if known (else nullptr)
  * @param[in] query_regions the image regions (type must be the same as the database)
  * @param[out] pose found pose
  * @param[out] resection_data matching data (2D-3D and inliers; optional)
  * @return True if a putative pose has been estimated
  */
  virtual bool Localize
  (
    const Pair & image_size,
    const cameras::IntrinsicBase * optional_intrinsics,
    const features::Regions & query_regions,
    geometry::Pose3 & pose,
    Image_Localizer_Match_Data * resection_data = nullptr // optional
  ) const = 0;


  /**
  * @brief Try to localize an image from known 2D-3D matches
  *
  * @param[in] image_size the w,h image size
  * @param[in] optional_intrinsics camera intrinsic if known (else nullptr)
  * @param[in,out] resection_data matching data (with filled 2D-3D correspondences). 
   * The 2D points are supposed to be the original distorted image points
  * @param[out] pose found pose
  * @param[in] estimator The type of robust estimator to use. The only supported 
   * frameworks are ROBUST_ESTIMATOR_ACRANSAC and ROBUST_ESTIMATOR_LORANSAC.
  * @return True if a putative pose has been estimated
  */
  static bool Localize
  (
    const Pair & image_size,
    const cameras::IntrinsicBase * optional_intrinsics,
    Image_Localizer_Match_Data & resection_data,
    geometry::Pose3 & pose,
    robust::EROBUST_ESTIMATOR estimator = robust::ROBUST_ESTIMATOR_ACRANSAC
  );

  /**
  * @brief Refine a pose according 2D-3D matching & camera model data
  *
  * @param[in,out] intrinsics Camera model
  * @param[in,out] pose Camera pose
  * @param[in] matching_data Corresponding 2D-3D data
  * @param[in] b_refine_pose tell if pose must be refined
  * @param[in] b_refine_intrinsic tell if intrinsics must be refined
  * @return True if the refinement decreased the RMSE pixel residual error
  */
  static bool RefinePose
  (
    cameras::IntrinsicBase * intrinsics,
    geometry::Pose3 & pose,
    const Image_Localizer_Match_Data & matching_data,
    bool b_refine_pose,
    bool b_refine_intrinsic
  );
};


} // namespace sfm
} // namespace openMVG
