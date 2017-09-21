// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/matching/matcherType.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matchingImageCollection/pairBuilder.hpp"
#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"

#include <string>
#include <vector>

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
  IImageCollectionMatcher() {};

  virtual ~IImageCollectionMatcher() {}

  /// Find corresponding points between some pair of view Ids
  virtual void Match(
    const sfm::SfMData & sfm_data,
    const feature::RegionsPerView& regionsPerView,
    const Pair_Set & pairs, // list of pair to consider for matching
    feature::EImageDescriberType descType,
    matching::PairwiseMatches & map_putatives_matches // the output pairwise photometric corresponding points
    ) const = 0;
};

} // namespace aliceVision
} // namespace matchingImageCollection
