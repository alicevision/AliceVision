
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_IND_MATCH_H
#define OPENMVG_MATCHING_IND_MATCH_H

#include "openMVG/types.hpp"
#include "openMVG/features/ImageDescriberCommon.hpp"

#include <cereal/cereal.hpp> // Serialization

#include <iostream>
#include <set>
#include <map>
#include <vector>

#define OPENMVG_DEBUG_MATCHING

namespace openMVG {
namespace matching {

/// Structure in order to save pairwise indexed references.
/// A sort operator exist in order to remove duplicates of IndMatch series.
struct IndMatch
{
  IndMatch(
          IndexT i = 0, IndexT j = 0
#ifdef OPENMVG_DEBUG_MATCHING
          , float distance = 0.0
#endif
          )
  {
    _i = i;
    _j = j;
#ifdef OPENMVG_DEBUG_MATCHING
    _distance = distance;
#endif
  }

  friend bool operator==(const IndMatch& m1, const IndMatch& m2)  {
    return (m1._i == m2._i && m1._j == m2._j);
  }

  friend bool operator!=(const IndMatch& m1, const IndMatch& m2)  {
    return !(m1 == m2);
  }

  // Lexicographical ordering of matches. Used to remove duplicates.
  friend bool operator<(const IndMatch& m1, const IndMatch& m2) {
    return (m1._i < m2._i || (m1._i == m2._i && m1._j < m2._j));
  }

  /// Remove duplicates ((_i, _j) that appears multiple times)
  static bool getDeduplicated(std::vector<IndMatch> & vec_match)
  {
    const size_t sizeBefore = vec_match.size();
    std::set<IndMatch> set_deduplicated( vec_match.begin(), vec_match.end());
    vec_match.assign(set_deduplicated.begin(), set_deduplicated.end());
    return sizeBefore != vec_match.size();
  }

  // Serialization
  template <class Archive>
  void serialize( Archive & ar )  {
    ar(_i, _j);
  }

  IndexT _i, _j;  // Left, right index
#ifdef OPENMVG_DEBUG_MATCHING
  float _distance;
#endif
};

static inline std::ostream& operator<<(std::ostream & out, const IndMatch & obj) {
  return out << obj._i << " " << obj._j;
}

static inline std::istream& operator>>(std::istream & in, IndMatch & obj) {
  return in >> obj._i >> obj._j;
}

typedef std::vector<matching::IndMatch> IndMatches;

struct MatchesPerDescType: public std::map<features::EImageDescriberType, IndMatches>
{
    int getNbMatches(features::EImageDescriberType descType) const
    {
        const auto& it = this->find(descType);
        if(it == this->end())
            return 0;
        return it->second.size();
    }
    int getNbAllMatches() const
    {
        int nbMatches = 0;
        for(const auto& matches: *this)
        {
            nbMatches += matches.second.size();
        }
        return nbMatches;
    }
};

/// Pairwise matches (indexed matches for a pair <I,J>)
/// The structure used to store corresponding point indexes per images pairs

typedef std::map<Pair, MatchesPerDescType> PairwiseMatches;

typedef std::map<Pair, IndMatches> PairwiseSimpleMatches;

inline Pair_Set getImagePairs(const PairwiseMatches & matches)
{
  Pair_Set pairs;
  for(PairwiseMatches::const_iterator it = matches.begin(); it != matches.end(); ++it)
    pairs.insert(it->first);
  return pairs;
}


}  // namespace matching
}  // namespace openMVG

#endif // OPENMVG_MATCHING_IND_MATCH_H
