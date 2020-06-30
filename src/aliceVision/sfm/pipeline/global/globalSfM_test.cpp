// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/utils/statistics.hpp>
#include <aliceVision/sfm/utils/syntheticScene.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/sfm/sfm.hpp>

#include <boost/filesystem.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE GLOBAL_SFM

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

namespace fs = boost::filesystem;

// Test summary:
// - Create features points and matching from the synthetic dataset
// - Init a SfMData scene VIew and Intrinsic from a synthetic dataset
// - Perform Global SfM on the data
// - Assert that:
//   - mean residual error is below the gaussian noise added to observation
//   - the desired number of tracks are found,
//   - the desired number of poses are found.
BOOST_AUTO_TEST_CASE(GLOBAL_SFM_RotationAveragingL2_TranslationAveragingL1)
{
  const int nviews = 6;
  const int npoints = 64;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfmData2,
    "./",
    "./Reconstruction_Report.html");

  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  generateSyntheticFeatures(featuresPerView, feature::EImageDescriberType::UNKNOWN, sfmData, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, sfmData, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK(residual < 0.5);
  BOOST_CHECK(sfmEngine.getSfMData().getPoses().size() == nviews);
  BOOST_CHECK(sfmEngine.getSfMData().getLandmarks().size() == npoints);
}

BOOST_AUTO_TEST_CASE(GLOBAL_SFM_RotationAveragingL1_TranslationAveragingL1)
{
  const int nviews = 6;
  const int npoints = 64;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfmData2,
    "./",
    "./Reconstruction_Report.html");

  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  generateSyntheticFeatures(featuresPerView, feature::EImageDescriberType::UNKNOWN, sfmData, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, sfmData, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L1);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L1);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK(residual < 0.5);
  BOOST_CHECK(sfmEngine.getSfMData().getPoses().size() == nviews);
  BOOST_CHECK(sfmEngine.getSfMData().getLandmarks().size() == npoints);
}

BOOST_AUTO_TEST_CASE(GLOBAL_SFM_RotationAveragingL2_TranslationAveragingL2_Chordal)
{
  const int nviews = 6;
  const int npoints = 64;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfmData2,
    "./",
    "./Reconstruction_Report.html");

  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  generateSyntheticFeatures(featuresPerView, feature::EImageDescriberType::UNKNOWN, sfmData, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, sfmData, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK(residual < 0.5);
  BOOST_CHECK(sfmEngine.getSfMData().getPoses().size() == nviews);
  BOOST_CHECK(sfmEngine.getSfMData().getLandmarks().size() == npoints);
}

BOOST_AUTO_TEST_CASE(GLOBAL_SFM_RotationAveragingL2_TranslationAveragingSoftL1)
{
  const int nviews = 6;
  const int npoints = 64;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_globalSfM sfmEngine(
    sfmData2,
    "./",
    "./Reconstruction_Report.html");

  // Add a tiny noise in 2D observations to make data more realistic
  std::normal_distribution<double> distribution(0.0,0.5);

  // Configure the featuresPerView & the matches_provider from the synthetic dataset
  feature::FeaturesPerView featuresPerView;
  generateSyntheticFeatures(featuresPerView, feature::EImageDescriberType::UNKNOWN, sfmData, distribution);

  matching::PairwiseMatches pairwiseMatches;
  generateSyntheticMatches(pairwiseMatches, sfmData, feature::EImageDescriberType::UNKNOWN);

  // Configure data provider (Features and Matches)
  sfmEngine.SetFeaturesProvider(&featuresPerView);
  sfmEngine.SetMatchesProvider(&pairwiseMatches);

  // Configure reconstruction parameters
  sfmEngine.setLockAllIntrinsics(true);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(ROTATION_AVERAGING_L2);
  sfmEngine.SetTranslationAveragingMethod(TRANSLATION_AVERAGING_SOFTL1);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK(residual < 0.5);
  BOOST_CHECK(sfmEngine.getSfMData().getPoses().size() == nviews);
  BOOST_CHECK(sfmEngine.getSfMData().getLandmarks().size() == npoints);
}
