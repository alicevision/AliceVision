// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "matchingCommon.hpp"

#include "aliceVision/matchingImageCollection/ImageCollectionMatcher_generic.hpp"
#include "aliceVision/matchingImageCollection/ImageCollectionMatcher_cascadeHashing.hpp"

#include <exception>
#include <cassert>

namespace aliceVision {
namespace matchingImageCollection {
  

std::unique_ptr<IImageCollectionMatcher> createImageCollectionMatcher(matching::EMatcherType matcherType, float distRatio, bool crossMatching)
{
  std::unique_ptr<IImageCollectionMatcher> matcherPtr;
  
  switch(matcherType)
  {
    case matching::BRUTE_FORCE_L2:          matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, crossMatching, matching::BRUTE_FORCE_L2)); break;
    case matching::ANN_L2:                  matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, crossMatching, matching::ANN_L2)); break;
    case matching::CASCADE_HASHING_L2:      matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, crossMatching, matching::CASCADE_HASHING_L2)); break;
    case matching::FAST_CASCADE_HASHING_L2: matcherPtr.reset(new ImageCollectionMatcher_cascadeHashing(distRatio)); break;
    case matching::BRUTE_FORCE_HAMMING:     matcherPtr.reset(new ImageCollectionMatcher_generic(distRatio, crossMatching, matching::BRUTE_FORCE_HAMMING)); break;
    
    default: throw std::out_of_range("Invalid matcherType enum");
  }
  assert(matcherPtr != nullptr);
  
  return matcherPtr;
}

} // namespace matchingImageCollection
} // namespace aliceVision
