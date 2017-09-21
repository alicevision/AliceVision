// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <utility>
#include <vector>

namespace aliceVision {
namespace translationAveraging {

/// Relative information [Rij|tij] for a pair
typedef std::pair< Pair, std::pair<Mat3,Vec3> > relativeInfo;

typedef std::vector< relativeInfo > RelativeInfoVec;
typedef std::map< Pair, std::pair<Mat3, Vec3> > RelativeInfoMap;

// List the pairs used by the relative motions
static Pair_Set getPairs(const RelativeInfoVec & vec_relative)
{
  Pair_Set pair_set;
  for(size_t i = 0; i < vec_relative.size(); ++i)
  {
    const relativeInfo & rel = vec_relative[i];
    pair_set.insert(Pair(rel.first.first, rel.first.second));
  }
  return pair_set;
}

// List the index used by the relative motions
static std::set<IndexT> getIndexT(const RelativeInfoVec & vec_relative)
{
  std::set<IndexT> indexT_set;
  for (RelativeInfoVec::const_iterator iter = vec_relative.begin();
    iter != vec_relative.end(); ++iter)
  {
    indexT_set.insert(iter->first.first);
    indexT_set.insert(iter->first.second);
  }
  return indexT_set;
}

} // namespace translationAveraging
} // namespace aliceVision
