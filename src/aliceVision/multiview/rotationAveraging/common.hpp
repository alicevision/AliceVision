// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/types.hpp>

#include <vector>
#include <map>

namespace aliceVision   {
namespace rotationAveraging  {

/// Representation of weighted relative rotations data between two poses
struct RelativeRotation {
  IndexT i, j; // pose's indices
  Mat3 Rij; // pose's relative rotation
  float weight;

  RelativeRotation(IndexT i_=0, IndexT j_=0, const	Mat3 & Rij_=Mat3::Identity(), float weight_=1.0f):
  i(i_), j(j_), Rij(Rij_), weight(weight_)
  {}
};

typedef std::vector<RelativeRotation> RelativeRotations;
typedef std::map<Pair, RelativeRotation> RelativeRotationsMap;

/// List the pairs used by the relative rotations
static PairSet getPairs(const RelativeRotations & relRots)
{
  PairSet pairs;
  for(RelativeRotations::const_iterator it = relRots.begin(); it != relRots.end(); ++it)
    pairs.insert(std::make_pair(it->i, it->j));
  return pairs;
}

/// Convert a relative motion iterable sequence to RelativeRotation indexed by pairs
static RelativeRotationsMap getMap(const RelativeRotations & relRots)
{
  RelativeRotationsMap map_rots;
  for(RelativeRotations::const_iterator it = relRots.begin(); it != relRots.end(); ++it)
    map_rots[std::make_pair(it->i, it->j)] = *it;
  return map_rots;
}

} // namespace rotationAveraging
} // namespace aliceVision
