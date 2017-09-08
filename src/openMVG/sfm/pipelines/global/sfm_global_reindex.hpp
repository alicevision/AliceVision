// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_SFM_GLOBAL_REINDEX_HPP
#define OPENMVG_SFM_GLOBAL_REINDEX_HPP

namespace aliceVision {
namespace sfm{

/// Association of Ids to a contiguous set of Ids
template<typename IterablePairs, typename PairValueType>
void reindex(
  const IterablePairs& pairs,
  Hash_Map<PairValueType, PairValueType> & _reindexForward,
  Hash_Map<PairValueType, PairValueType> & _reindexBackward)
{
  typedef std::pair<PairValueType,PairValueType> PairT;
  // get an unique set of Ids
  std::set<size_t> _uniqueId;
  for(typename IterablePairs::const_iterator iter = pairs.begin();
        iter != pairs.end(); ++iter)
  {
    _uniqueId.insert(iter->first);
    _uniqueId.insert(iter->second);
  }

  // Build the Forward and Backward mapping
  for(typename IterablePairs::const_iterator iter = pairs.begin();
        iter != pairs.end(); ++iter)
  {
    if (_reindexForward.find(iter->first) == _reindexForward.end())
    {
      const size_t dist = std::distance(_uniqueId.begin(), _uniqueId.find(iter->first));
      _reindexForward[iter->first] = dist;
      _reindexBackward[dist] = iter->first;
    }
    if (_reindexForward.find(iter->second) == _reindexForward.end())
    {
      const size_t dist = std::distance(_uniqueId.begin(), _uniqueId.find(iter->second));
      _reindexForward[iter->second] = dist;
      _reindexBackward[dist] = iter->second;
    }
  }
}

} // namespace sfm
} // namespace aliceVision

#endif // OPENMVG_SFM_GLOBAL_REINDEX_HPP
