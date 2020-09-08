// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/matching/IndMatch.hpp>

namespace aliceVision {
namespace sfm {

class StructureEstimationFromKnownPoses
{
public:

  /// Use geometry of the views to compute a putative structure from features and descriptors.
  void run(sfmData::SfMData& sfmData,
    const PairSet& pairs,
    const feature::RegionsPerView& regionsPerView,
    std::mt19937 &randomNumberGenerator, 
    double geometricErrorMax);

public:

  /// Use guided matching to find corresponding 2-view correspondences
  void match(const sfmData::SfMData& sfmData,
    const PairSet& pairs,
    const feature::RegionsPerView& regionsPerView,
    double geometricErrorMax);

  /// Filter inconsistent correspondences by using 3-view correspondences on view triplets
  void filter(
    const sfmData::SfMData& sfmData,
    const PairSet& pairs,
    const feature::RegionsPerView& regionsPerView);

  /// Init & triangulate landmark observations from validated 3-view correspondences
  void triangulate(
    sfmData::SfMData& sfmData,
    const feature::RegionsPerView& regionsPerView,
    std::mt19937 &randomNumberGenerator);

  const matching::PairwiseMatches& getPutativesMatches() const { return _putativeMatches; }

private:
  //--
  // DATA (temporary)
  //--
  matching::PairwiseMatches _putativeMatches;
  matching::PairwiseMatches _tripletMatches;
};

} // namespace sfm
} // namespace aliceVision

