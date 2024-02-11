// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace aliceVision {

typedef uint32_t IndexT;
static const IndexT UndefinedIndexT = std::numeric_limits<IndexT>::max();

typedef std::pair<IndexT, IndexT> Pair;
typedef std::set<Pair> PairSet;
typedef std::vector<Pair> PairVec;

struct EstimationStatus
{
    EstimationStatus(bool valid, bool strongSupport)
      : isValid(valid),
        hasStrongSupport(strongSupport)
    {}

    bool isValid = false;
    bool hasStrongSupport = false;
};

/**
 * @brief Defines the state of a parameter for an estimator
 */
enum class EEstimatorParameterState : std::uint8_t
{
    REFINED = 0,   //< will be adjusted by the estimator
    CONSTANT = 1,  //< will be set as constant in the estimator
    IGNORED = 2    //< will not be set into the estimator
};

}  // namespace aliceVision
