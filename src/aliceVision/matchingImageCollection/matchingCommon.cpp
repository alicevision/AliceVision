// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "matchingCommon.hpp"

#include "aliceVision/matchingImageCollection/ImageCollectionMatcher_generic.hpp"
#include "aliceVision/matchingImageCollection/ImageCollectionMatcher_cascadeHashing.hpp"

#include <exception>
#include <cassert>

namespace aliceVision {
namespace matchingImageCollection {
  

std::unique_ptr<IImageCollectionMatcher> createImageCollectionMatcher(matching::EMatcherType matcherType, float distRatio)
{
  std::unique_ptr<IImageCollectionMatcher> matcherPtr;
  
  switch(matcherType)
  {
    case matching::BRUTE_FORCE_L2:          matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, matching::BRUTE_FORCE_L2)); break;
    case matching::ANN_L2:                  matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, matching::ANN_L2)); break;
    case matching::CASCADE_HASHING_L2:      matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, matching::CASCADE_HASHING_L2)); break;
    case matching::FAST_CASCADE_HASHING_L2: matcherPtr.reset(new ImageCollectionMatcher_cascadeHashing(distRatio)); break;
    case matching::BRUTE_FORCE_HAMMING:     matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, matching::BRUTE_FORCE_HAMMING)); break;
    
    default: throw std::out_of_range("Invalid matcherType enum");
  }
  assert(matcherPtr != nullptr);
  
  return matcherPtr;
}

} // namespace matchingImageCollection
} // namespace aliceVision
