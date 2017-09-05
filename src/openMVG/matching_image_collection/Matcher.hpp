// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/matching/matcher_type.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/matching_image_collection/Pair_Builder.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/features/RegionsPerView.hpp"

#include <string>
#include <vector>

namespace openMVG {
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

} // namespace openMVG
} // namespace matching_image_collection
