// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/matching/matcherType.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matchingImageCollection/pairBuilder.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"

#include <string>
#include <vector>
#include <random>

namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief Image Collection Matcher interface.
 *
 * Compute putative matches between a collection of pictures
 */
class IImageCollectionMatcher
{
  public:
  IImageCollectionMatcher() = default;

  virtual ~IImageCollectionMatcher() = default;

  /// Find corresponding points between some pair of view Ids
  virtual void Match(
    std::mt19937 & randomNumberGenerator,
    const feature::RegionsPerView& regionsPerView,
    const PairSet & pairs, // list of pair to consider for matching
    feature::EImageDescriberType descType,
    matching::PairwiseMatches & map_putatives_matches // the output pairwise photometric corresponding points
    ) const = 0;
};

} // namespace aliceVision
} // namespace matchingImageCollection
