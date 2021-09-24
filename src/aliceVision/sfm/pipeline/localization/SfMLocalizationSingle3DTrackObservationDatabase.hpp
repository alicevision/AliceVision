// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/localization/SfMLocalizer.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>

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
  * @param[in] sfmData the SfM scene that have to be described
  * @param[in] regionPerView regions provider
  * @return True if the database has been correctly setup
  */
  bool Init(const sfmData::SfMData& sfmData,
            const feature::RegionsPerView& regionsPerView) override;

  /**
  * @brief Try to localize an image in the database
  *
  * @param[in] imageSize the w,h image size
  * @param[in] optionalIntrinsics camera intrinsic if known (else nullptr)
  * @param[in] queryRegions the image regions (type must be the same as the database)
  * @param[in] randomNumberGenerator the random number generator
  * @param[out] pose found pose
  * @param[out] resectionData matching data (2D-3D and inliers; optional)
  * @return True if a putative pose has been estimated
  */
  bool Localize(const Pair& imageSize,
                const camera::IntrinsicBase* optionalIntrinsics,
                const feature::Regions& queryRegions,
                std::mt19937 &randomNumberGenerator, 
                geometry::Pose3& pose,
                ImageLocalizerMatchData* resectionDataPtr = nullptr // optional
                ) const override;

private:
  // Reference to the scene
  const sfmData::SfMData* _sfmData;
  /// Association of a regions to a landmark observation
  std::unique_ptr<feature::Regions> _landmarkObservationsDescriptors;
  /// Association of a track observation to a track Id (used for retrieval)
  std::vector<IndexT> _indexToLandmarkId;
  /// A matching interface to find matches between 2D descriptor matches
  ///  and 3D points observation descriptors
  std::unique_ptr<matching::RegionsDatabaseMatcher> _matchingInterface;
};

} // namespace sfm
} // namespace aliceVision
