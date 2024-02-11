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
inline std::set<IndexT> getIndexes(const IterableIndexTSequence& seq)
{
    std::set<IndexT> setOut;
    for (typename IterableIndexTSequence::const_iterator it = seq.begin(); it != seq.end(); ++it)
        setOut.insert(it->first);
    return setOut;
}

/// Filter the toFilter iterable sequence (keep only the element that share a common index
///  with the provided Ids index list).
template<typename T>
inline void keepOnlyReferencedElement(const std::set<IndexT>& Ids, T& toFilter)
{
    ALICEVISION_LOG_ERROR("Must be specialized for your type");
}

// Specialization for RelativeInfoMap
inline void keepOnlyReferencedElement(const std::set<IndexT>& setRemainingIds, translationAveraging::RelativeInfoMap& mapRelatives)
{
    translationAveraging::RelativeInfoMap mapRelativesInfered;
    for (translationAveraging::RelativeInfoMap::const_iterator iter = mapRelatives.begin(); iter != mapRelatives.end(); ++iter)
    {
        if (setRemainingIds.find(iter->first.first) != setRemainingIds.end() && setRemainingIds.find(iter->first.second) != setRemainingIds.end())
        {
            mapRelativesInfered.insert(*iter);
        }
    }
    mapRelatives.swap(mapRelativesInfered);
}

// Specialization for RelativeInfoMap
template<>
inline void keepOnlyReferencedElement(const std::set<IndexT>& setRemainingIds, rotationAveraging::RelativeRotations& relativeInfo)
{
    rotationAveraging::RelativeRotations relativesInfered;
    for (rotationAveraging::RelativeRotations::const_iterator iter = relativeInfo.begin(); iter != relativeInfo.end(); ++iter)
    {
        if (setRemainingIds.find(iter->i) != setRemainingIds.end() && setRemainingIds.find(iter->j) != setRemainingIds.end())
        {
            relativesInfered.push_back(*iter);
        }
    }
    relativeInfo.swap(relativesInfered);
}

// Specialization for PairwiseMatches
template<>
inline void keepOnlyReferencedElement(const std::set<IndexT>& setRemainingIds, aliceVision::matching::PairwiseMatches& mapMatches)
{
    aliceVision::matching::PairwiseMatches mapMatchesEInfered;
    for (aliceVision::matching::PairwiseMatches::const_iterator iter = mapMatches.begin(); iter != mapMatches.end(); ++iter)
    {
        if (setRemainingIds.find(iter->first.first) != setRemainingIds.end() && setRemainingIds.find(iter->first.second) != setRemainingIds.end())
        {
            mapMatchesEInfered.insert(*iter);
        }
    }
    mapMatches.swap(mapMatchesEInfered);
}

// Specialization for std::map<IndexT,Mat3>
template<>
inline void keepOnlyReferencedElement(const std::set<IndexT>& setRemainingIds, std::map<IndexT, Mat3>& mapMat3)
{
    std::map<IndexT, Mat3> mapInfered;
    for (std::map<IndexT, Mat3>::const_iterator iter = mapMat3.begin(); iter != mapMat3.end(); ++iter)
    {
        if (setRemainingIds.find(iter->first) != setRemainingIds.end())
        {
            mapInfered.insert(*iter);
        }
    }
    mapMat3.swap(mapInfered);
}

// Specialization for RelativeInfoVec
template<>
inline void keepOnlyReferencedElement(const std::set<IndexT>& setRemainingIds, translationAveraging::RelativeInfoVec& relativeInfoVec)
{
    translationAveraging::RelativeInfoVec mapInfered;
    for (translationAveraging::RelativeInfoVec::const_iterator iter = relativeInfoVec.begin(); iter != relativeInfoVec.end(); ++iter)
    {
        if (setRemainingIds.find(iter->first.first) != setRemainingIds.end() && setRemainingIds.find(iter->first.second) != setRemainingIds.end())
        {
            mapInfered.push_back(*iter);
        }
    }
    relativeInfoVec.swap(mapInfered);
}

}  // namespace sfm
}  // namespace aliceVision
