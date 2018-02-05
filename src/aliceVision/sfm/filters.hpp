// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/rotationAveraging/common.hpp>
#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace sfm { 

template<typename IterableIndexTSequence>
inline std::set<IndexT> getIndexes(const IterableIndexTSequence & seq)
{
  std::set<IndexT> setOut;
  for(typename IterableIndexTSequence::const_iterator it = seq.begin(); it != seq.end(); ++it)
    setOut.insert(it->first);
  return setOut;
}

/// Filter the toFilter iterable sequence (keep only the element that share a common index
///  with the provided Ids index list).
template<typename T>
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & Ids,
  T & toFilter)
{
  ALICEVISION_LOG_ERROR("Must be specialized for your type");
}

// Specialization for RelativeInfoMap
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  translationAveraging::RelativeInfoMap& map_relatives)
{
  translationAveraging::RelativeInfoMap map_relatives_infered;
  for (translationAveraging::RelativeInfoMap::const_iterator
    iter = map_relatives.begin();
    iter != map_relatives.end(); ++iter)
  {
    if (set_remainingIds.find(iter->first.first) != set_remainingIds.end() &&
        set_remainingIds.find(iter->first.second) != set_remainingIds.end())
    {
      map_relatives_infered.insert(*iter);
    }
  }
  map_relatives.swap(map_relatives_infered);
}

// Specialization for RelativeInfoMap
template<>
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  rotationAveraging::RelativeRotations& relative_info)
{
  rotationAveraging::RelativeRotations relatives_infered;
  for (rotationAveraging::RelativeRotations::const_iterator
    iter = relative_info.begin();
    iter != relative_info.end(); ++iter)
  {
    if (set_remainingIds.find(iter->i) != set_remainingIds.end() &&
        set_remainingIds.find(iter->j) != set_remainingIds.end())
    {
      relatives_infered.push_back(*iter);
    }
  }
  relative_info.swap(relatives_infered);
}

// Specialization for PairwiseMatches
template<>
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  aliceVision::matching::PairwiseMatches& map_matches)
{
  aliceVision::matching::PairwiseMatches map_matches_E_infered;
  for (aliceVision::matching::PairwiseMatches::const_iterator iter = map_matches.begin();
    iter != map_matches.end(); ++iter)
  {
    if (set_remainingIds.find(iter->first.first) != set_remainingIds.end() &&
        set_remainingIds.find(iter->first.second) != set_remainingIds.end())
    {
      map_matches_E_infered.insert(*iter);
    }
  }
  map_matches.swap(map_matches_E_infered);
}

// Specialization for std::map<IndexT,Mat3>
template<>
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  std::map<IndexT,Mat3>& map_Mat3)
{
  std::map<IndexT,Mat3> map_infered;
  for (std::map<IndexT,Mat3>::const_iterator iter = map_Mat3.begin();
    iter != map_Mat3.end(); ++iter)
  {
    if (set_remainingIds.find(iter->first) != set_remainingIds.end())
    {
      map_infered.insert(*iter);
    }
  }
  map_Mat3.swap(map_infered);
}

// Specialization for RelativeInfoVec
template<>
inline void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  translationAveraging::RelativeInfoVec & relativeInfo_vec)
{
  translationAveraging::RelativeInfoVec map_infered;
  for (translationAveraging::RelativeInfoVec::const_iterator iter = relativeInfo_vec.begin();
    iter != relativeInfo_vec.end(); ++iter)
  {
    if (set_remainingIds.find(iter->first.first) != set_remainingIds.end() &&
        set_remainingIds.find(iter->first.second) != set_remainingIds.end())
    {
      map_infered.push_back(*iter);
    }
  }
  relativeInfo_vec.swap(map_infered);
}

} // namespace sfm
} // namespace aliceVision
