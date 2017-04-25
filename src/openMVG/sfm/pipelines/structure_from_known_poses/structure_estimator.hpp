// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/sfm/sfm_data.hpp"

#include "openMVG/features/RegionsPerView.hpp"
#include "openMVG/matching/indMatch.hpp"

namespace openMVG {
namespace sfm {

class SfM_Data_Structure_Estimation_From_Known_Poses
{
public:

  /// Use geometry of the views to compute a putative structure from features and descriptors.
  void run(
    SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const features::RegionsPerView& regionsPerView);

public:

  /// Use guided matching to find corresponding 2-view correspondences
  void match(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const features::RegionsPerView& regionsPerView);

  /// Filter inconsistent correspondences by using 3-view correspondences on view triplets
  void filter(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const features::RegionsPerView& regionsPerView);

  /// Init & triangulate landmark observations from validated 3-view correspondences
  void triangulate(
    SfM_Data & sfm_data,
    const features::RegionsPerView& regionsPerView);

private:
  //--
  // DATA (temporary)
  //--
  matching::PairwiseMatches _putativeMatches;
  matching::PairwiseMatches _tripletMatches;
};

} // namespace sfm
} // namespace openMVG

