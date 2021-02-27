// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/matching/matcherType.hpp"
#include "aliceVision/matchingImageCollection/IImageCollectionMatcher.hpp"

namespace aliceVision {
namespace matchingImageCollection {
  
/**
 * 
 * @param matcherType
 * @return 
 */
std::unique_ptr<IImageCollectionMatcher> createImageCollectionMatcher(matching::EMatcherType matcherType, float distRatio, bool crossMatching);


} // namespace matching
} // namespace aliceVision