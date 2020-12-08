// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/graph/graph.hpp>

namespace aliceVision {
namespace sfm {

enum ETranslationAveragingMethod
{
  TRANSLATION_AVERAGING_L1 = 1,
  TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL = 2,
  TRANSLATION_AVERAGING_SOFTL1 = 3
};

inline std::string ETranslationAveragingMethod_enumToString(ETranslationAveragingMethod eTranslationAveragingMethod)
{
  switch(eTranslationAveragingMethod)
  {
    case ETranslationAveragingMethod::TRANSLATION_AVERAGING_L1:
      return "L1_minimization";
    case ETranslationAveragingMethod::TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL:
      return "L2_minimization";
  case ETranslationAveragingMethod::TRANSLATION_AVERAGING_SOFTL1:
    return "L1_soft_minimization";
  }
  throw std::out_of_range("Invalid translation averaging method type");
}

inline ETranslationAveragingMethod ETranslationAveragingMethod_stringToEnum(const std::string& TranslationAveragingMethodName)
{
  if(TranslationAveragingMethodName == "L1_minimization")      return ETranslationAveragingMethod::TRANSLATION_AVERAGING_L1;
  if(TranslationAveragingMethodName == "L2_minimization")   return ETranslationAveragingMethod::TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL;
  if(TranslationAveragingMethodName == "L1_soft_minimization")      return ETranslationAveragingMethod::TRANSLATION_AVERAGING_SOFTL1;

  throw std::out_of_range("Invalid translation averaging method name : '" + TranslationAveragingMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ETranslationAveragingMethod e)
{
    return os << ETranslationAveragingMethod_enumToString(e);
}

inline std::istream& operator>>(std::istream& in, ETranslationAveragingMethod& translationType)
{
    std::string token;
    in >> token;
    translationType = ETranslationAveragingMethod_stringToEnum(token);
    return in;
}

class GlobalSfMTranslationAveragingSolver
{
  translationAveraging::RelativeInfoVec m_vec_initialRijTijEstimates;

public:

  /**
   * @brief Use features in normalized camera frames
   */
  bool Run(ETranslationAveragingMethod eTranslationAveragingMethod,
           sfmData::SfMData& sfmData,
           const feature::FeaturesPerView& normalizedFeaturesPerView,
           const matching::PairwiseMatches& pairwiseMatches,
           const HashMap<IndexT, Mat3>& map_globalR,
           std::mt19937 & randomNumberGenerator,
           matching::PairwiseMatches& tripletWise_matches);

private:
  bool Translation_averaging(ETranslationAveragingMethod eTranslationAveragingMethod,
           sfmData::SfMData& sfmData,
           const HashMap<IndexT, Mat3> & map_globalR);

  void Compute_translations(const sfmData::SfMData& sfmData,
           const feature::FeaturesPerView& normalizedFeaturesPerView,
           const matching::PairwiseMatches& pairwiseMatches,
           const HashMap<IndexT, Mat3>& map_globalR,
           std::mt19937 & randomNumberGenerator,
           matching::PairwiseMatches& tripletWise_matches);

  /**
   * @brief Compute the relative translations on the rotations graph.
   * Compute relative translations by using triplets of poses.
   * Use an edge coverage algorithm to reduce the graph covering complexity
   * Complexity: sub-linear in term of edges count.
   */
  void ComputePutativeTranslation_EdgesCoverage(const sfmData::SfMData& sfmData,
           const HashMap<IndexT, Mat3>& map_globalR,
           const feature::FeaturesPerView& normalizedFeaturesPerView,
           const matching::PairwiseMatches& pairwiseMatches,
           std::mt19937 & randomNumberGenerator,
           translationAveraging::RelativeInfoVec& vec_initialEstimates,
           matching::PairwiseMatches& newpairMatches);

  /**
   * @brief Robust estimation and refinement of a translation and 3D points of an image triplets.
   */
  bool Estimate_T_triplet(const sfmData::SfMData& sfmData,
           const HashMap<IndexT, Mat3>& map_globalR,
           const feature::FeaturesPerView& normalizedFeaturesPerView,
           const matching::PairwiseMatches& pairwiseMatches,
           const graph::Triplet& poses_id,
           std::mt19937 & randomNumberGenerator,
           std::vector<Vec3>& vec_tis,
           double& precision, // UpperBound of the precision found by the AContrario estimator
           std::vector<size_t>& vec_inliers,
           aliceVision::track::TracksMap& rig_tracks,
           const std::string& outDirectory) const;
};

} // namespace sfm
} // namespace aliceVision
