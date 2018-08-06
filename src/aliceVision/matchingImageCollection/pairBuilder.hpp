// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <algorithm>

namespace aliceVision {

/// Generate all the (I,J) pairs of the upper diagonal of the NxN matrix
PairSet exhaustivePairs(const sfmData::Views& views, int rangeStart=-1, int rangeSize=0);

/// Load a set of PairSet from a file
/// I J K L (pair that link I)
bool loadPairs(
     const std::string &sFileName, // filename of the list file,
     PairSet & pairs,
     int rangeStart=-1,
     int rangeSize=0);

/// Save a set of PairSet to a file (one pair per line)
/// I J
/// I K
bool savePairs(const std::string &sFileName, const PairSet & pairs);

}; // namespace aliceVision
