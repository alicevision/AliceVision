// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/features/ImageDescriberCommon.hpp"
#include "aliceVision/matching/matcher_type.hpp"
#include "aliceVision/matching/indMatch.hpp"
#include "aliceVision/matching_image_collection/Pair_Builder.hpp"
#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/features/RegionsPerView.hpp"

#include <string>
#include <vector>

namespace aliceVision {
namespace matching_image_collection {

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
    const sfm::SfM_Data & sfm_data,
    const features::RegionsPerView& regionsPerView,
    const Pair_Set & pairs, // list of pair to consider for matching
    features::EImageDescriberType descType,
    matching::PairwiseMatches & map_putatives_matches // the output pairwise photometric corresponding points
    ) const = 0;
};

} // namespace aliceVision
} // namespace matching_image_collection
