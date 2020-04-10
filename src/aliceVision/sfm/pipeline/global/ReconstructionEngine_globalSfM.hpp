// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/pipeline/ReconstructionEngine.hpp>
#include <aliceVision/sfm/pipeline/global/GlobalSfMRotationAveragingSolver.hpp>
#include <aliceVision/sfm/pipeline/global/GlobalSfMTranslationAveragingSolver.hpp>

#include <dependencies/htmlDoc/htmlDoc.hpp>

namespace aliceVision{
namespace sfm{

/// Global SfM Pipeline Reconstruction Engine.
/// - Method: Global Fusion of Relative Motions.
class ReconstructionEngine_globalSfM : public ReconstructionEngine
{
public:

  ReconstructionEngine_globalSfM(const sfmData::SfMData& sfmData,
                                 const std::string& outDirectory,
                                 const std::string& loggingFile = "");

  ~ReconstructionEngine_globalSfM();

  void SetFeaturesProvider(feature::FeaturesPerView* featuresPerView);
  void SetMatchesProvider(matching::PairwiseMatches* provider);

  void SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod);
  void SetTranslationAveragingMethod(ETranslationAveragingMethod eTranslationAveragingMethod);

  void setLockAllIntrinsics(bool v) { _lockAllIntrinsics = v; }

  virtual bool process();

protected:
  /// Compute from relative rotations the global rotations of the camera poses
  bool Compute_Global_Rotations(const aliceVision::rotationAveraging::RelativeRotations& vec_relatives_R,
                                HashMap<IndexT, Mat3>& map_globalR);

  /// Compute/refine relative translations and compute global translations
  bool Compute_Global_Translations(const HashMap<IndexT, Mat3>& global_rotations,
                                   matching::PairwiseMatches& tripletWise_matches);

  /// Compute the initial structure of the scene
  bool Compute_Initial_Structure(matching::PairwiseMatches& tripletWise_matches);

  /// Adjust the scene (& remove outliers)
  bool Adjust();

private:
  /// Compute relative rotations
  void Compute_Relative_Rotations(aliceVision::rotationAveraging::RelativeRotations& vec_relatives_R);

  // Logger
  std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  std::string _loggingFile;

  // Parameter
  ERotationAveragingMethod _eRotationAveragingMethod;
  ETranslationAveragingMethod _eTranslationAveragingMethod;
  bool _lockAllIntrinsics = false;
  EFeatureConstraint _featureConstraint = EFeatureConstraint::BASIC;

  // Data provider
  feature::FeaturesPerView* _featuresPerView;
  matching::PairwiseMatches* _pairwiseMatches;

  std::shared_ptr<feature::FeaturesPerView> _normalizedFeaturesPerView;
};

} // namespace sfm
} // namespace aliceVision
