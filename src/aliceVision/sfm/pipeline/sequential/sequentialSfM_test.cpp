// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/feature/imageDescriberCommon.hpp"
#include "aliceVision/sfm/pipeline/syntheticScene.hpp"
#include "aliceVision/sfm/sfm.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE SEQUENTIAL_SFM
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

// Test summary:
// - Create features points and matching from the synthetic dataset
// - Init a SfMData scene VIew and Intrinsic from a synthetic dataset
// - Perform Sequential SfM on the data
// - Assert that:
//   - mean residual error is below the gaussian noise added to observation
//   - the desired number of tracks are found,
//   - the desired number of poses are found.

// Test a scene where all the camera intrinsics are known
BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Known_Intrinsics) {

  const int nviews = 6;
  const int npoints = 128;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_sequentialSfM sfmEngine(
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Set an initial pair
  sfmEngine.setInitialPair(Pair(0,1));

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  BOOST_CHECK (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  BOOST_CHECK( dResidual < 0.5);
  BOOST_CHECK_EQUAL(sfmEngine.Get_SfMData().GetPoses().size(), nviews);
  BOOST_CHECK_EQUAL(sfmEngine.Get_SfMData().GetLandmarks().size(), npoints);
}

// Test a scene where only the two first camera have known intrinsics
BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Partially_Known_Intrinsics) {

  const int nviews = 6;
  const int npoints = 256;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();
  // Only the first two views will have valid intrinsics
  // Remaining one will have undefined intrinsics
  for (Views::iterator iterV = sfm_data_2.views.begin();
    iterV != sfm_data_2.views.end(); ++iterV)
  {
    if (std::distance(sfm_data_2.views.begin(),iterV) >1)
    {
      iterV->second.get()->setIntrinsicId(UndefinedIndexT);
    }
  }

  ReconstructionEngine_sequentialSfM sfmEngine(
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Set an initial pair
  sfmEngine.setInitialPair(Pair(0,1));

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  BOOST_CHECK (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  BOOST_CHECK( dResidual < 0.5);
  BOOST_CHECK_EQUAL(nviews, sfmEngine.Get_SfMData().GetPoses().size());
  BOOST_CHECK_EQUAL(npoints, sfmEngine.Get_SfMData().GetLandmarks().size());
}

BOOST_AUTO_TEST_CASE(SEQUENTIAL_SFM_Rig) {

  const int nviews = 10;
  const int npoints = 128;
  const std::size_t nbSubPoses = 2;
  const std::size_t nbPoses = nviews / nbSubPoses;

  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  const SfMData sfm_data = getInputRigScene(d, config, PINHOLE_CAMERA, nbSubPoses, nbPoses);

  // Remove poses and structure
  SfMData sfm_data_2 = sfm_data;
  sfm_data_2.GetPoses().clear();
  sfm_data_2.structure.clear();

  ReconstructionEngine_sequentialSfM sfmEngine(
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
  sfmEngine.setFeatures(&featuresPerView);
  sfmEngine.setMatches(&pairwiseMatches);

  // Set an initial pair
  sfmEngine.setInitialPair(Pair(0,1));

  // Configure reconstruction parameters
  sfmEngine.Set_bFixedIntrinsics(true);

  BOOST_CHECK (sfmEngine.Process());

  const double dResidual = RMSE(sfmEngine.Get_SfMData());
  ALICEVISION_LOG_DEBUG("RMSE residual: " << dResidual);
  BOOST_CHECK( dResidual < 0.5);
  BOOST_CHECK( sfmEngine.Get_SfMData().GetPoses().size() == nbPoses);
  BOOST_CHECK( sfmEngine.Get_SfMData().GetLandmarks().size() == npoints);
}
