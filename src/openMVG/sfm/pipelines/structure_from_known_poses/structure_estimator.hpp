// Copyright (c) 2015 Pierre Moulon.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/sfm/pipelines/RegionsPerView.hpp"
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
    const RegionsPerView& regionsPerView);

public:

  /// Use guided matching to find corresponding 2-view correspondences
  void match(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const RegionsPerView& regionsPerView);

  /// Filter inconsistent correspondences by using 3-view correspondences on view triplets
  void filter(
    const SfM_Data & sfm_data,
    const Pair_Set & pairs,
    const RegionsPerView& regionsPerView);

  /// Init & triangulate landmark observations from validated 3-view correspondences
  void triangulate(
    SfM_Data & sfm_data,
    const RegionsPerView& regionsPerView);

private:
  //--
  // DATA (temporary)
  //--
  matching::PairWiseSimpleMatches putatives_matches;
  matching::PairWiseSimpleMatches triplets_matches;
};

} // namespace sfm
} // namespace openMVG

