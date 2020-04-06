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

#ifdef ALICEVISION_STD_UNORDERED_MAP
#include <unordered_map>
#endif

namespace aliceVision {

typedef uint32_t IndexT;
static const IndexT UndefinedIndexT = std::numeric_limits<IndexT>::max();

typedef std::pair<IndexT,IndexT> Pair;
typedef std::set<Pair> PairSet;
typedef std::vector<Pair> PairVec;

#ifdef ALICEVISION_UNORDERED_MAP
template<typename Key, typename Value>
using HashMap = std::unordered_map<Key, Value>;
#else
template<typename K, typename V>
using HashMap = std::map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<const K,V> > >;
#endif


struct EstimationStatus
{
  EstimationStatus(bool valid, bool strongSupport)
    : isValid(valid)
    , hasStrongSupport(strongSupport)
  {}

  bool isValid = false;
  bool hasStrongSupport = false;
};

} // namespace aliceVision
