// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>
#include <aliceVision/sfm/utils/syntheticScene.hpp>
#include <aliceVision/sfm/sfm.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE SEQUENTIAL_SFM

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

// Test summary:
// - Create features points and matching from the synthetic dataset
// - Init a SfMData scene VIew and Intrinsic from a synthetic dataset
// - Perform Sequential SfM on the data
// - Assert that:
//   - mean residual error is below the gaussian noise added to observation
//   - the desired number of tracks are found,
//   - the desired number of poses are found.

// Test a scene where all the camera intrinsics are known
BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Known_Intrinsics)
{
  const int nviews = 6;
  const int npoints = 128;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_sequentialSfM::Params sfmParams;
  sfmParams.userInitialImagePair = Pair(0, 1);
  sfmParams.lockAllIntrinsics = true;

  ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData2,
    sfmParams,
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK_LT(residual, 0.5);
  BOOST_CHECK_EQUAL(sfmEngine.getSfMData().getPoses().size(), nviews);
  BOOST_CHECK_EQUAL(sfmEngine.getSfMData().getLandmarks().size(), npoints);
}

// Test a scene where only the two first camera have known intrinsics
BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Partially_Known_Intrinsics)
{
  const int nviews = 6;
  const int npoints = 256;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  // The first two views will have valid intrinsics.
  // The third one will have an invalid intrinsic (unknown focal length)
  {
    // Create the intrinsic with unknown focal length
    sfmData2.intrinsics[1] = std::make_shared<camera::Pinhole>
        (config._cx*2, config._cy*2, -1, -1, 0, 0);
    // The 3rd view use this invalid intrinsic
    sfmData2.views[2]->setIntrinsicId(1);
  }

  ReconstructionEngine_sequentialSfM::Params sfmParams;
  // sfmParams.userInitialImagePair = Pair(0, 1); // test automatic selection of initial pair
  sfmParams.lockAllIntrinsics = true;

  ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData2,
    sfmParams,
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  BOOST_CHECK (sfmEngine.process());

  const SfMData& finalSfMData = sfmEngine.getSfMData();
  const double residual = RMSE(finalSfMData);
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK_LT(residual, 0.5);
  BOOST_CHECK_EQUAL(nviews, finalSfMData.getPoses().size());
  BOOST_CHECK_EQUAL(npoints, finalSfMData.getLandmarks().size());
  BOOST_CHECK_NE(reinterpret_cast<const camera::Pinhole*>(finalSfMData.getIntrinsics().at(0).get())->getFocalLengthPixX(),
                 reinterpret_cast<const camera::Pinhole*>(finalSfMData.getIntrinsics().at(1).get())->getFocalLengthPixX());
}

BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Known_Rig)
{
  const int nbPoses = 10;
  const int nbPoints = 128;

  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nbPoses, nbPoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfmData = getInputRigScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfmData2 = sfmData;
  sfmData2.getPoses().clear();
  sfmData2.structure.clear();

  ReconstructionEngine_sequentialSfM::Params sfmParams;
  sfmParams.userInitialImagePair = Pair(0, 2);
  sfmParams.lockAllIntrinsics = true;

  ReconstructionEngine_sequentialSfM sfmEngine(
    sfmData2,
    sfmParams,
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  BOOST_CHECK (sfmEngine.process());

  const double residual = RMSE(sfmEngine.getSfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << residual);
  BOOST_CHECK_LT(residual, 0.5);
  BOOST_CHECK_EQUAL(sfmEngine.getSfMData().getPoses().size(), nbPoses);
  BOOST_CHECK_EQUAL(sfmEngine.getSfMData().getLandmarks().size(), nbPoints);
}

