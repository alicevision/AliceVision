// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_SFM_GLOBAL_ENGINE_RELATIVE_MOTIONS_HPP
#define ALICEVISION_SFM_GLOBAL_ENGINE_RELATIVE_MOTIONS_HPP

#include "aliceVision/sfm/pipelines/sfm_engine.hpp"

#include "aliceVision/sfm/pipelines/global/GlobalSfM_rotation_averaging.hpp"
#include "aliceVision/sfm/pipelines/global/GlobalSfM_translation_averaging.hpp"
#include "third_party/htmlDoc/htmlDoc.hpp"

namespace aliceVision{
namespace sfm{

/// Global SfM Pipeline Reconstruction Engine.
/// - Method: Global Fusion of Relative Motions.
class GlobalSfMReconstructionEngine_RelativeMotions : public ReconstructionEngine
{
public:

  GlobalSfMReconstructionEngine_RelativeMotions(
    const SfM_Data & sfm_data,
    const std::string & soutDirectory,
    const std::string & loggingFile = "");

  ~GlobalSfMReconstructionEngine_RelativeMotions();

  void SetFeaturesProvider(features::FeaturesPerView * featuresPerView);
  void SetMatchesProvider(matching::PairwiseMatches * provider);

  void SetRotationAveragingMethod(ERotationAveragingMethod eRotationAveragingMethod);
  void SetTranslationAveragingMethod(ETranslationAveragingMethod _eTranslationAveragingMethod);

  virtual bool Process();

protected:
  /// Compute from relative rotations the global rotations of the camera poses
  bool Compute_Global_Rotations
  (
    const aliceVision::rotation_averaging::RelativeRotations & vec_relatives_R,
    Hash_Map<IndexT, Mat3> & map_globalR
  );

  /// Compute/refine relative translations and compute global translations
  bool Compute_Global_Translations
  (
    const Hash_Map<IndexT, Mat3> & global_rotations,
    matching::PairwiseMatches & tripletWise_matches
  );

  /// Compute the initial structure of the scene
  bool Compute_Initial_Structure
  (
    matching::PairwiseMatches & tripletWise_matches
  );

  // Adjust the scene (& remove outliers)
  bool Adjust();

private:
  /// Compute relative rotations
  void Compute_Relative_Rotations
  (
    aliceVision::rotation_averaging::RelativeRotations & vec_relatives_R
  );

  //----
  //-- Data
  //----

  // HTML logger
  std::shared_ptr<htmlDocument::htmlDocumentStream> _htmlDocStream;
  std::string _sLoggingFile;

  // Parameter
  ERotationAveragingMethod _eRotationAveragingMethod;
  ETranslationAveragingMethod _eTranslationAveragingMethod;

  //-- Data provider
  features::FeaturesPerView  * _featuresPerView;
  matching::PairwiseMatches  * _pairwiseMatches;

  std::shared_ptr<features::FeaturesPerView> _normalizedFeaturesPerView;
};

} // namespace sfm
} // namespace aliceVision

#endif // ALICEVISION_SFM_GLOBAL_ENGINE_RELATIVE_MOTIONS_HPP
