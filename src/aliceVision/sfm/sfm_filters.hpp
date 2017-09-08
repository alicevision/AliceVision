// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_SFM_FILTERS_HPP
#define ALICEVISION_SFM_FILTERS_HPP

#include <aliceVision/multiview/rotation_averaging_common.hpp>
#include <aliceVision/multiview/translation_averaging_common.hpp>
#include <aliceVision/matching/indMatch.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace sfm { 

template<typename IterableIndexTSequence>
static std::set<IndexT> getIndexes(const IterableIndexTSequence & seq)
{
  std::set<IndexT> setOut;
  for(typename IterableIndexTSequence::const_iterator it = seq.begin(); it != seq.end(); ++it)
    setOut.insert(it->first);
  return setOut;
}

/// Filter the toFilter iterable sequence (keep only the element that share a common index
///  with the provided Ids index list).
template<typename T>
static void KeepOnlyReferencedElement(
  const std::set<IndexT> & Ids,
  T & toFilter)
{
  ALICEVISION_LOG_ERROR("Must be specialized for your type");
}

// Specialization for RelativeInfo_Map
template<>
#ifdef _MSC_VER
static
#endif
void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  RelativeInfo_Map& map_relatives)
{
  RelativeInfo_Map map_relatives_infered;
  for (RelativeInfo_Map::const_iterator
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

// Specialization for RelativeInfo_Map
template<>
#ifdef _MSC_VER
static
#endif
void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  rotation_averaging::RelativeRotations& relative_info)
{
  rotation_averaging::RelativeRotations relatives_infered;
  for (rotation_averaging::RelativeRotations::const_iterator
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
#ifdef _MSC_VER
static
#endif
void KeepOnlyReferencedElement(
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
#ifdef _MSC_VER
static
#endif
void KeepOnlyReferencedElement(
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

// Specialization for RelativeInfo_Vec
template<>
#ifdef _MSC_VER
static
#endif
void KeepOnlyReferencedElement(
  const std::set<IndexT> & set_remainingIds,
  RelativeInfo_Vec & relativeInfo_vec)
{
  RelativeInfo_Vec map_infered;
  for (RelativeInfo_Vec::const_iterator iter = relativeInfo_vec.begin();
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

#endif // ALICEVISION_SFM_FILTERS_HPP
