// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/pipeline/syntheticScene.hpp"
#include "aliceVision/feature/FeaturesPerView.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "testing/testing.h"

#include <cmath>
#include <cstdio>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

// Test summary:
// - Create features points and matching from the synthetic dataset
// - Init a SfMData scene VIew and Intrinsic from a synthetic dataset
// - Perform Global SfM on the data
// - Assert that:
//   - mean residual error is below the gaussian noise added to observation
//   - the desired number of tracks are found,
//   - the desired number of poses are found.
TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(feature::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL1_TranslationAveragingL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(feature::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L1);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingL2_Chordal) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(feature::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingSoftL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
   featuresPerView.createSyntheticData(feature::EImageDescriberType::UNKNOWN, d, distribution);

   matching::PairwiseMatches pairwiseMatches;
   generateSyntheticMatches(pairwiseMatches, d, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_SOFTL1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfMData().GetLandmarks().size() == npoints);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
