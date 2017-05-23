
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

//-----------------
// Test summary:
//-----------------
// - Create features points and matching from the synthetic dataset
// - Init a SfM_Data scene VIew and Intrinsic from a synthetic dataset
// - Perform Global SfM on the data
// - Assert that:
//   - mean residual error is below the gaussian noise added to observation
//   - the desired number of tracks are found,
//   - the desired number of poses are found.
//-----------------

#include "openMVG/sfm/pipelines/pipelines_test.hpp"
#include "openMVG/features/FeaturesPerView.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/sfm/sfm.hpp"
#include "testing/testing.h"

#include <cmath>
#include <cstdio>
#include <iostream>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::sfm;


TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfM_Data scene
  const SfM_Data sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfM_Data sfm_data_2 = sfm_data;
  sfm_data_2.poses.clear();
  sfm_data_2.structure.clear();

  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  features::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(features::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, features::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfM_Data());
  OPENMVG_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL1_TranslationAveragingL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfM_Data scene
  const SfM_Data sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfM_Data sfm_data_2 = sfm_data;
  sfm_data_2.poses.clear();
  sfm_data_2.structure.clear();

  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  features::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(features::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, features::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L1);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfM_Data());
  OPENMVG_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingL2_Chordal) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfM_Data scene
  const SfM_Data sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfM_Data sfm_data_2 = sfm_data;
  sfm_data_2.poses.clear();
  sfm_data_2.structure.clear();

  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  features::FeaturesPerView featuresPerView;
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
  featuresPerView.createSyntheticData(features::EImageDescriberType::UNKNOWN, d, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, d, features::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfM_Data());
  OPENMVG_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetLandmarks().size() == npoints);
}

TEST(GLOBAL_SFM, RotationAveragingL2_TranslationAveragingSoftL1) {

  const int nviews = 6;
  const int npoints = 64;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfM_Data scene
  const SfM_Data sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfM_Data sfm_data_2 = sfm_data;
  sfm_data_2.poses.clear();
  sfm_data_2.structure.clear();

  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data_2,
    "./",
    stlplus::create_filespec("./", "Reconstruction_Report.html"));

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  features::FeaturesPerView featuresPerView;
  
  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);
   featuresPerView.createSyntheticData(features::EImageDescriberType::UNKNOWN, d, distribution);

   matching::PairwiseMatches pairwiseMatches;
   generateSyntheticMatches(pairwiseMatches, d, features::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_SOFTL1);

  EXPECT_TRUE (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfM_Data());
  OPENMVG_LOG_DEBUG("RMSE residual: " << dResidual);
  EXPECT_TRUE( dResidual < 0.5);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetPoses().size() == nviews);
  EXPECT_TRUE( sfmEngine.Get_SfM_Data().GetLandmarks().size() == npoints);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
