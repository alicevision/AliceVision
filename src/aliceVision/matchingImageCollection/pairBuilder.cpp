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
PairSet exhaustivePairs(const std::set<IndexT> & viewIds)
{
    PairSet pairs;
    auto itA = viewIds.begin();
    auto itAEnd = viewIds.end();

    for (; itA != itAEnd; ++itA)
    {
        auto itB = itA;
        std::advance(itB, 1);

        for (; itB != viewIds.end(); ++itB)
        {
            pairs.insert(std::make_pair(*itA, *itB));
        }
    }

    return pairs;
}

};  // namespace aliceVision
