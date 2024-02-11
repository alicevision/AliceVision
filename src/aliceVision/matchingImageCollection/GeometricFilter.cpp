// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GeometricFilter.hpp"

namespace aliceVision {
namespace matchingImageCollection {

void removePoorlyOverlappingImagePairs(PairwiseMatches& geometricMatches,
                                       const PairwiseMatches& putativeMatches,
                                       float minimumRatio,
                                       std::size_t minimumGeometricCount)
{
    std::vector<PairwiseMatches::key_type> toRemoveVec;
    for (const auto& match : geometricMatches)
    {
        const size_t photometricCount = putativeMatches.find(match.first)->second.getNbAllMatches();
        const size_t geometricCount = match.second.getNbAllMatches();
        const float ratio = geometricCount / (float)photometricCount;
        if (geometricCount < minimumGeometricCount || ratio < minimumRatio)
        {
            toRemoveVec.push_back(match.first);  // the image pair will be removed
        }
    }

    // remove discarded pairs
    for (const auto& pair : toRemoveVec)
    {
        geometricMatches.erase(pair);
    }
}

}  // namespace matchingImageCollection
}  // namespace aliceVision
