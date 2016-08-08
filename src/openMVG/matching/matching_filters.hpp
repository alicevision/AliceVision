
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_MATCHINGFILTERS_H
#define OPENMVG_MATCHING_MATCHINGFILTERS_H

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/features/regions.hpp"
#include <algorithm>
#include <cassert>
#include <iterator>
#include <set>
#include <vector>

namespace openMVG {
namespace matching {

using namespace std;

/**
  * Nearest neighbor distance ratio filtering ( a < fratio * b) :
  * Ratio between best and second best matches must be superior to
  * given threshold. It avoid matches that have similar descriptor,
  * (so it removes potential ambiguities).
  *
  * \param[in]  first    Iterator on the sequence of distance.
  * \param[in]  last     Iterator of the end of the sequence.
  * \param[in]  NN       Number of neighbor in iterator
  *   sequence (minimum required 2).
  * \param[out] vec_ratioOkIndex  Indexes that respect NN dist Ratio)
  * \param[in]  fratio            Ratio value (default value 0.6f)
  *
  * \return void.
  */
template <typename DataInputIterator>
static inline void NNdistanceRatio
(
  DataInputIterator first, // distance start
  DataInputIterator last,  // distance end
  int NN, // Number of neighbor in iterator sequence (minimum required 2)
  std::vector<int> & vec_ratioOkIndex, // output (index that respect NN dist Ratio)
  float fratio = 0.6f) // ratio value
{
  assert( NN >= 2);

  const size_t n = std::distance(first,last);
  vec_ratioOkIndex.clear();
  vec_ratioOkIndex.reserve(n/NN);
  DataInputIterator iter = first;
  for(size_t i=0; i < n/NN; ++i, std::advance(iter, NN))
  {
    DataInputIterator iter2 = iter;
    std::advance(iter2, 1);
    if ( (*iter) < fratio * (*iter2))
        vec_ratioOkIndex.push_back(static_cast<int>(i));
  }
}

// Consider the match valid if the 2 first matches correspond to the "same" feature.
//
// In [Mishkin 2015] "MODS: Fast and Robust Method for Two-View Matching" they introduce the FGINN strategy
// "First Geometrically Inconsistent Nearest Neighbor".
// Here, for simplicity and performances, we don't search for the first
// geometrically inconsistent but assume that the match is strong enough if
// the 2 nearest neighbors are on the same image region.
// It could be 2 really close features or the same feature with multiple
// descriptors (different orientations of SIFT for instance).
// Warning: This code assumes that NNN__ is 2.
template <typename DataInputIterator>
static inline void NNdistanceRatio_multiDesc
(
  DataInputIterator first, // distance start
  DataInputIterator last,  // distance end
  int NN, // Number of neighbor in iterator sequence (minimum required 2)
  const features::Regions* regions,
  const matching::IndMatches& vec_nIndice,
  std::vector<int> & vec_ratioOkIndex, // output (index that respect NN dist Ratio)
  float fratio = 0.6f) // ratio value
{
  assert( NN >= 2);

  const size_t n = std::distance(first,last);
  vec_ratioOkIndex.clear();
  vec_ratioOkIndex.reserve(n/NN);
  DataInputIterator iter = first;
  for(size_t i=0; i < n/NN; ++i, std::advance(iter, NN))
  {
    const Vec2 vecA = regions->GetRegionPosition(vec_nIndice[i*NN]._j);
    DataInputIterator iter2 = iter;
    std::advance(iter2, 1);
    bool validMatch = true;
    for(size_t j=0; j < NN; ++j, ++iter2)
    {
      const Vec2 vecB = regions->GetRegionPosition(vec_nIndice[i*NN+j]._j);
      if(std::abs(vecA.x() - vecB.x()) < 10.0 &&
         std::abs(vecA.y() - vecB.y()) < 10.0)
      {
        continue;
      }
      validMatch = false;
      if ((*iter) < 90000 && (*iter) < fratio * (*iter2))
      {
        validMatch = true;
        break;
      }
    }
    if(validMatch)
      vec_ratioOkIndex.push_back(static_cast<int>(i));
  }
}


/**
  * Symmetric matches filtering :
  * Suppose matches from dataset A to B stored in vec_matches
  * Suppose matches from dataset B to A stored in vec_reversematches
  * A matches is kept if (i == vec_reversematches[vec_matches[i]])
  * If NN > 1 => Only the major matches are considered.
  *
  * \param[in]  first    matches from A to B.
  * \param[in]  last     matches from B to A.
  * \param[in]  NN       Number of neighbor matches.
  * \param[out] vec_goodIndex  Indexes that respect Symmetric matches
  *
  * \return void.
  */
// TODO
static inline void SymmetricMatches(const vector<int> & vec_matches,
  const vector<int> & vec_reversematches,
  int NN,
  vector<int> & vec_goodIndex)
{
  assert (NN >= 1);

  int index = 0;
  for (size_t i=0; i<vec_matches.size(); i+=NN, ++index)
  {
    // Add the match only if we have a symmetric result.
    if (index == vec_reversematches[vec_matches[i]*NN])  {
      vec_goodIndex.push_back(index);
    }
  }
}

/**
  * Intersect two container via Iterator.
  * Keep element that exist in the two sequence of data.
  *
  * \param[in]  aStart  Begin iterator on the sequence A.
  * \param[in]  aEnd    End iterator on sequence A.
  * \param[in]  bStart  Begin iterator on the sequence B.
  * \param[in]  bEnd    End iterator on sequence B.
  * \param[out] vec_out Merged output indexes.
  *
  * \return void.
  */
template <typename Iterator, typename Type>
static inline void IntersectMatches( Iterator aStart, Iterator aEnd,
                       Iterator bStart, Iterator bEnd,
                       vector<Type> & vec_out)
{
  //-- Compute the intersection of the two vector
  //--- Use STL to perform it. Require that the input vectors are sorted.
  std::set<Type> intersect;
  std::set_intersection( aStart, aEnd,
                         bStart, bEnd,
                         std::inserter( intersect, intersect.begin() ) );

  vec_out = vector<Type>(intersect.begin(), intersect.end());
}

enum eMatchFilter
{
  MATCHFILTER_SYMMETRIC = 1,
  MATCHFILTER_NNDISTANCERATIO = 2,
  MATCHFILER_SYM_AND_NNDISTANCERATIO = MATCHFILTER_SYMMETRIC | MATCHFILTER_NNDISTANCERATIO
};

static inline void Filter( int NN,
       const vector<int> & vec_Matches01,
       const vector<float> & vec_distance01,
       const vector<int> & vec_Matches10,
       const vector<float> & vec_distance10,
       vector<IndMatch> & vec_outIndex,
       eMatchFilter matchFilter,
       float fNNDistanceRatio = 0.6f)
{
  vector<int> vec_symmetricIndex, vec_NNDistRatioIndexes;

  if (matchFilter == MATCHFILTER_SYMMETRIC ||
      matchFilter == MATCHFILER_SYM_AND_NNDISTANCERATIO)
  {
    SymmetricMatches(vec_Matches01,
      vec_Matches10,
      NN,
      vec_symmetricIndex);
  }

  if (matchFilter == MATCHFILTER_NNDISTANCERATIO ||
      matchFilter == MATCHFILER_SYM_AND_NNDISTANCERATIO)
  {
    if ( NN == 1)
    {
      vec_NNDistRatioIndexes = vec_Matches01;
    }

    NNdistanceRatio( vec_distance01.begin(), // distance start
      vec_distance01.end(),  // distance end
      NN, // Number of neighbor in iterator sequence (minimum required 2)
      vec_NNDistRatioIndexes, // output (index that respect Lowe Ratio)
      fNNDistanceRatio);
  }

  switch (matchFilter)
  {
  case MATCHFILTER_NNDISTANCERATIO:

    for (size_t i=0; i < vec_NNDistRatioIndexes.size()-1&& vec_NNDistRatioIndexes.size()>0; ++i)
    {
      vec_outIndex.push_back( IndMatch(vec_NNDistRatioIndexes[i], vec_Matches01[vec_NNDistRatioIndexes[i]*NN]) );
    }

    break;

  case MATCHFILTER_SYMMETRIC:

    for (size_t i=0; i < vec_symmetricIndex.size()-1&& vec_symmetricIndex.size()>0; ++i)
    {
      vec_outIndex.push_back( IndMatch(vec_symmetricIndex[i], vec_Matches01[vec_symmetricIndex[i]*NN]) );
    }

    break;

  case MATCHFILER_SYM_AND_NNDISTANCERATIO:

    vector<int> vec_indexes;
    //-- Compute the intersection of the two vector
    IntersectMatches(vec_symmetricIndex.begin(), vec_symmetricIndex.end(),
      vec_NNDistRatioIndexes.begin(), vec_NNDistRatioIndexes.end(),
      vec_indexes);

    for (size_t i=0; i < vec_indexes.size()-1 && vec_indexes.size()>0; ++i)
      vec_outIndex.push_back( IndMatch(vec_indexes[i], vec_Matches01[vec_indexes[i]*NN]) );

    break;
  }

  // Remove multi-index
  {
    std::sort(vec_outIndex.begin(), vec_outIndex.end());
    std::vector<IndMatch>::iterator end = std::unique(vec_outIndex.begin(), vec_outIndex.end());
    if(end != vec_outIndex.end()) {
      vec_outIndex.erase(end, vec_outIndex.end());
    }
  }
}

}  // namespace matching
}  // namespace openMVG

#endif // OPENMVG_MATCHING_MATCHINGFILTERS_H
