// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/sfm/SfMData.hpp"
#include "aliceVision/multiview/translationAveraging/common.hpp"
#include "aliceVision/feature/FeaturesPerView.hpp"
#include "aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp"
#include "aliceVision/track/Track.hpp"
#include "aliceVision/graph/graph.hpp"

namespace aliceVision{
namespace sfm{

enum ETranslationAveragingMethod
{
  TRANSLATION_AVERAGING_L1 = 1,
  TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL = 2,
  TRANSLATION_AVERAGING_SOFTL1 = 3
};

class GlobalSfMTranslationAveragingSolver
{
  translationAveraging::RelativeInfoVec m_vec_initialRijTijEstimates;

public:

  /// Use features in normalized camera frames
  bool Run(
    ETranslationAveragingMethod eTranslationAveragingMethod,
    SfMData & sfm_data,
    const feature::FeaturesPerView & normalizedFeaturesPerView,
    const matching::PairwiseMatches & matches_provider,
    const HashMap<IndexT, Mat3> & map_globalR,
    matching::PairwiseMatches & tripletWise_matches
  );

private:
  bool Translation_averaging(
    ETranslationAveragingMethod eTranslationAveragingMethod,
    SfMData & sfm_data,
    const HashMap<IndexT, Mat3> & map_globalR);

  void Compute_translations(
    const SfMData & sfm_data,
    const feature::FeaturesPerView & normalizedFeaturesPerView,
    const matching::PairwiseMatches & matches_provider,
    const HashMap<IndexT, Mat3> & map_globalR,
    matching::PairwiseMatches &tripletWise_matches);

  //-- Compute the relative translations on the rotations graph.
  // Compute relative translations by using triplets of poses.
  // Use an edge coverage algorithm to reduce the graph covering complexity
  // Complexity: sub-linear in term of edges count.
  void ComputePutativeTranslation_EdgesCoverage(
    const SfMData & sfm_data,
    const HashMap<IndexT, Mat3> & map_globalR,
    const feature::FeaturesPerView & normalizedFeaturesPerView,
    const matching::PairwiseMatches & matches_provider,
    translationAveraging::RelativeInfoVec & vec_initialEstimates,
    matching::PairwiseMatches & newpairMatches);

  // Robust estimation and refinement of a translation and 3D points of an image triplets.
  bool Estimate_T_triplet(
    const SfMData & sfm_data,
    const HashMap<IndexT, Mat3> & map_globalR,
    const feature::FeaturesPerView & normalizedFeaturesPerView,
    const matching::PairwiseMatches & matches_provider,
    const graph::Triplet & poses_id,
    std::vector<Vec3> & vec_tis,
    double & dPrecision, // UpperBound of the precision found by the AContrario estimator
    std::vector<size_t> & vec_inliers,
    aliceVision::track::TracksMap & rig_tracks,
    const std::string & sOutDirectory) const;
};

} // namespace sfm
} // namespace aliceVision
