// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>

#include <iostream>
#include <vector>
#include <set>
#include <map>

#define ALICEVISION_DEBUG_MATCHING

namespace aliceVision {
namespace matching {

struct MatchesPerDescType : public std::map<feature::EImageDescriberType, IndMatches>
{
    int getNbMatches(feature::EImageDescriberType descType) const
    {
        const auto& it = this->find(descType);
        if (it == this->end())
            return 0;
        return it->second.size();
    }
    int getNbAllMatches() const
    {
        int nbMatches = 0;
        for (const auto& matches : *this)
        {
            nbMatches += matches.second.size();
        }
        return nbMatches;
    }
};

/// Pairwise matches (indexed matches for a pair <I,J>)
/// The structure used to store corresponding point indexes per images pairs

typedef std::map<Pair, MatchesPerDescType> PairwiseMatches;

typedef std::map<Pair, IndMatches> PairwiseSimpleMatches;

inline PairSet getImagePairs(const PairwiseMatches& matches)
{
    PairSet pairs;
    for (PairwiseMatches::const_iterator it = matches.begin(); it != matches.end(); ++it)
        pairs.insert(it->first);
    return pairs;
}

}  // namespace matching
}  // namespace aliceVision
