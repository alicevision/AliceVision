// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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
inline PairSet getPairs(const RelativeInfoVec & vec_relative)
{
  PairSet pair_set;
  for(size_t i = 0; i < vec_relative.size(); ++i)
  {
    const relativeInfo & rel = vec_relative[i];
    pair_set.insert(Pair(rel.first.first, rel.first.second));
  }
  return pair_set;
}

// List the index used by the relative motions
inline std::set<IndexT> getIndexT(const RelativeInfoVec & vec_relative)
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
