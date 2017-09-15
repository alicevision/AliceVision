// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/sfm/sfm_data.hpp"

#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matching/indMatch.hpp"

namespace aliceVision {
namespace sfm {

class SfM_Data_Structure_Estimation_From_Known_Poses
{
public:

  /// Use geometry of the views to compute a putative structure from features and descriptors.
  void run(
    SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const feature::RegionsPerView& regionsPerView);

public:

  /// Use guided matching to find corresponding 2-view correspondences
  void match(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const feature::RegionsPerView& regionsPerView);

  /// Filter inconsistent correspondences by using 3-view correspondences on view triplets
  void filter(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const feature::RegionsPerView& regionsPerView);

  /// Init & triangulate landmark observations from validated 3-view correspondences
  void triangulate(
    SfM_Data & sfm_data,
    const feature::RegionsPerView& regionsPerView);

private:
  //--
  // DATA (temporary)
  //--
  matching::PairwiseMatches _putativeMatches;
  matching::PairwiseMatches _tripletMatches;
};

} // namespace sfm
} // namespace aliceVision

