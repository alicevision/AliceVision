// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/feature/FeaturesPerView.hpp"
#include "aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"

namespace aliceVision {
namespace sfm {

// Implementation of a naive method:
// - init the database of descriptor from the structure and the observations.
// - create a large array with all the used descriptors and init a Matcher with it
// - to localize an input image compare it's regions to the database and robust estimate
//   the pose from found 2d-3D correspondences

class SfMLocalizationSingle3DTrackObservationDatabase : public SfMLocalizer
{
public:

  SfMLocalizationSingle3DTrackObservationDatabase();

  /**
  * @brief Build the retrieval database (3D points descriptors)
  *
  * @param[in] sfm_data the SfM scene that have to be described
  * @param[in] regionPerView regions provider
  * @return True if the database has been correctly setup
  */
  bool Init
  (
    const SfMData & sfm_data,
    const feature::RegionsPerView & regionsPerView
  );

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
  bool Localize
  (
    const Pair & image_size,
    const camera::IntrinsicBase * optional_intrinsics,
    const feature::Regions & query_regions,
    geometry::Pose3 & pose,
    ImageLocalizerMatchData * resection_data_ptr = nullptr // optional
  ) const;

private:
  // Reference to the scene
  const SfMData * sfm_data_;
  /// Association of a regions to a landmark observation
  std::unique_ptr<feature::Regions> landmark_observations_descriptors_;
  /// Association of a track observation to a track Id (used for retrieval)
  std::vector<IndexT> index_to_landmark_id_;
  /// A matching interface to find matches between 2D descriptor matches
  ///  and 3D points observation descriptors
  std::unique_ptr<matching::RegionsDatabaseMatcher> matching_interface_;
};

} // namespace sfm
} // namespace aliceVision
