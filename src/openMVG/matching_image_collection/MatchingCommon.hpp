// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "openMVG/matching/matcher_type.hpp"
#include "openMVG/matching_image_collection/Matcher.hpp"

namespace openMVG {
namespace matching_image_collection {
  
/**
 * 
 * @param matcherType
 * @return 
 */
std::unique_ptr<IImageCollectionMatcher> createImageCollectionMatcher(matching::EMatcherType matcherType, float distRatio);


} // namespace matching
} // namespace openMVG