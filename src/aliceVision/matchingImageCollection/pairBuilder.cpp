// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "pairBuilder.hpp"

#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string.hpp>

#include <set>
#include <iostream>
#include <fstream>
#include <sstream>

namespace aliceVision {

/// Generate all the (I,J) pairs of the upper diagonal of the NxN matrix
PairSet exhaustivePairs(const sfmData::Views& views, int rangeStart, int rangeSize)
{
    PairSet pairs;
    sfmData::Views::const_iterator itA = views.begin();
    sfmData::Views::const_iterator itAEnd = views.end();

    // If we have a rangeStart, only compute the matching for (rangeStart, X).
    if (rangeStart != -1 && rangeSize != 0)
    {
        if (rangeStart >= views.size())
            return pairs;
        std::advance(itA, rangeStart);
        itAEnd = views.begin();
        std::advance(itAEnd, std::min(std::size_t(rangeStart + rangeSize), views.size()));
    }

    for (; itA != itAEnd; ++itA)
    {
        sfmData::Views::const_iterator itB = itA;
        std::advance(itB, 1);
        for (; itB != views.end(); ++itB)
            pairs.insert(std::make_pair(itA->first, itB->first));
    }
    return pairs;
}

};  // namespace aliceVision
