// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
struct HashMap : std::unordered_map<Key, Value> {};
#else
template<typename K, typename V>
struct HashMap : std::map<K, V, std::less<K>,
 Eigen::aligned_allocator<std::pair<const K,V> > > {};
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
