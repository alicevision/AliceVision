// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace sfm{

/// association of Ids to a contiguous set of Ids
template<typename IterablePairs, typename PairValueType>
void reindex(const IterablePairs& pairs,
             HashMap<PairValueType, PairValueType>& _reindexForward,
             HashMap<PairValueType, PairValueType>& _reindexBackward)
{
  typedef std::pair<PairValueType,PairValueType> PairT;

  // get an unique set of Ids
  std::set<std::size_t> _uniqueId;

  for(typename IterablePairs::const_iterator iter = pairs.begin(); iter != pairs.end(); ++iter)
  {
    _uniqueId.insert(iter->first);
    _uniqueId.insert(iter->second);
  }

  // build the Forward and Backward mapping
  for(typename IterablePairs::const_iterator iter = pairs.begin(); iter != pairs.end(); ++iter)
  {
    if(_reindexForward.find(iter->first) == _reindexForward.end())
    {
      const size_t dist = std::distance(_uniqueId.begin(), _uniqueId.find(iter->first));
      _reindexForward[iter->first] = dist;
      _reindexBackward[dist] = iter->first;
    }

    if(_reindexForward.find(iter->second) == _reindexForward.end())
    {
      const size_t dist = std::distance(_uniqueId.begin(), _uniqueId.find(iter->second));
      _reindexForward[iter->second] = dist;
      _reindexBackward[dist] = iter->second;
    }
  }
}

} // namespace sfm
} // namespace aliceVision
