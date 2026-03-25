// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

#include <algorithm>

namespace aliceVision {

/**
 * @brief Generate all the (I,J) pairs of the upper diagonal of the NxN matrix
 * @param views the sfmData views indices list
 * @return a generated set of pairs
 */
PairSet exhaustivePairs(const std::set<IndexT> & viewIds);

};  // namespace aliceVision
