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