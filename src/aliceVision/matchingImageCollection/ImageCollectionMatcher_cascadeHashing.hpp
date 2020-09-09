// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/matchingImageCollection/IImageCollectionMatcher.hpp"

namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief Compute putative matches between a collection of pictures.
 *
 * Spurious correspondences are discarded by using the
 * a threshold over the distance ratio of the 2 nearest neighbours.
 *
 * @note: Cascade hashing tables are computed once and used for all the regions.
 * @warning: all descriptors are loaded in memory. You need to ensure that it can fit in RAM.
 */
class ImageCollectionMatcher_cascadeHashing : public IImageCollectionMatcher
{
  public:
  ImageCollectionMatcher_cascadeHashing
  (
    float dist_ratio
  );

  /// Find corresponding points between some pair of view Ids
  void Match(
    std::mt19937 & randomNumberGenerator,
    const feature::RegionsPerView& regionsPerView,
    const PairSet & pairs,
    feature::EImageDescriberType descType,
    matching::PairwiseMatches & map_PutativesMatches // the pairwise photometric corresponding points
  ) const;

  private:
  // Distance ratio used to discard spurious correspondence
  float f_dist_ratio_;
};

} // namespace aliceVision
} // namespace matchingImageCollection
