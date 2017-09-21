// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/multiview/test_data_sets.hpp"
#include "aliceVision/sfm/sfm.hpp"

#include "testing/testing.h"
#include "../camera/cameraCommon.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;

double RMSE(const SfMData & sfm_data);

SfMData getInputScene(const NViewDataSet & d, const nViewDatasetConfigurator & config, EINTRINSIC eintrinsic);

// Test summary:
// - Create a SfMData scene from a synthetic dataset
//   - since random noise have been added on 2d data point (initial residual is not small)
// - Check that residual is small once the generic Bundle Adjustment framework have been called.
// - Perform the test for all the plausible intrinsic camera models

TEST(BUNDLE_ADJUSTMENT, EffectiveMinimization_Pinhole) {

  const int nviews = 3;
  const int npoints = 6;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA);

  const double dResidual_before = RMSE(sfm_data);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  EXPECT_TRUE( ba_object->Adjust(sfm_data) );

  const double dResidual_after = RMSE(sfm_data);
  EXPECT_TRUE( dResidual_before > dResidual_after);
}

TEST(BUNDLE_ADJUSTMENT, EffectiveMinimization_PinholeRadialK1) {

  const int nviews = 3;
  const int npoints = 6;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA_RADIAL1);

  const double dResidual_before = RMSE(sfm_data);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  EXPECT_TRUE( ba_object->Adjust(sfm_data) );

  const double dResidual_after = RMSE(sfm_data);
  EXPECT_TRUE( dResidual_before > dResidual_after);
}

TEST(BUNDLE_ADJUSTMENT, EffectiveMinimization_PinholeRadialK3) {

  const int nviews = 3;
  const int npoints = 6;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA_RADIAL3);

  const double dResidual_before = RMSE(sfm_data);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  EXPECT_TRUE( ba_object->Adjust(sfm_data) );

  const double dResidual_after = RMSE(sfm_data);
  EXPECT_TRUE( dResidual_before > dResidual_after);
}

TEST(BUNDLE_ADJUSTMENT, EffectiveMinimization_PinholeBrownT2) {

  const int nviews = 3;
  const int npoints = 6;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA_BROWN);

  const double dResidual_before = RMSE(sfm_data);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  EXPECT_TRUE( ba_object->Adjust(sfm_data) );

  const double dResidual_after = RMSE(sfm_data);
  EXPECT_TRUE( dResidual_before > dResidual_after);
}

TEST(BUNDLE_ADJUSTMENT, EffectiveMinimization_PinholeFisheye) {

  const int nviews = 3;
  const int npoints = 6;
  const nViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfm_data = getInputScene(d, config, PINHOLE_CAMERA_FISHEYE);

  const double dResidual_before = RMSE(sfm_data);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  EXPECT_TRUE( ba_object->Adjust(sfm_data) );

  const double dResidual_after = RMSE(sfm_data);
  EXPECT_TRUE( dResidual_before > dResidual_after);
}

/// Compute the Root Mean Square Error of the residuals
double RMSE(const SfMData & sfm_data)
{
  // Compute residuals for each observation
  std::vector<double> vec;
  for(Landmarks::const_iterator iterTracks = sfm_data.GetLandmarks().begin();
      iterTracks != sfm_data.GetLandmarks().end();
      ++iterTracks)
  {
    const Observations & observations = iterTracks->second.observations;
    for(Observations::const_iterator itObs = observations.begin();
      itObs != observations.end(); ++itObs)
    {
      const View * view = sfm_data.GetViews().find(itObs->first)->second.get();
      const Pose3 pose = sfm_data.getPose(*view);
      const std::shared_ptr<IntrinsicBase> intrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId())->second;
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);
      vec.push_back( residual(0) );
      vec.push_back( residual(1) );
    }
  }
  const Eigen::Map<Eigen::RowVectorXd> residuals(&vec[0], vec.size());
  const double RMSE = std::sqrt(residuals.squaredNorm() / vec.size());
  return RMSE;
}

// Translation a synthetic scene into a valid SfMData scene.
// => A synthetic scene is used:
//    a random noise between [-.5,.5] is added on observed data points
SfMData getInputScene(const NViewDataSet & d, const nViewDatasetConfigurator & config, EINTRINSIC eintrinsic)
{
  // Translate the input dataset to a SfMData scene
  SfMData sfm_data;

  // 1. Views
  // 2. Poses
  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  // 4. Landmarks

  const int nviews = d._C.size();
  const int npoints = d._X.cols();

  // 1. Views
  for (int i = 0; i < nviews; ++i)
  {
    const IndexT id_view = i, id_pose = i, id_intrinsic = 0; //(shared intrinsics)
    sfm_data.views[i] = std::make_shared<View>("", id_view, id_intrinsic, id_pose, config._cx *2, config._cy *2);
  }

  // 2. Poses
  for (int i = 0; i < nviews; ++i)
  {
    Pose3 pose(d._R[i], d._C[i]);
    sfm_data.setPose(*sfm_data.views.at(i), pose);
  }

  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx *2;
    const unsigned int h = config._cy *2;
    sfm_data.intrinsics[0] = createPinholeIntrinsic(eintrinsic, w, h, config._fx, config._cx, config._cy);
  }

  // 4. Landmarks
  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Landmark landmark;
    landmark.X = d._X.col(i);
    for (int j = 0; j < nviews; ++j) {
      Vec2 pt = d._x[j].col(i);
      // => random noise between [-.5,.5] is added
      pt(0) += rand()/RAND_MAX - .5;
      pt(1) += rand()/RAND_MAX - .5;

      landmark.observations[j] = Observation(pt, i);
    }
    sfm_data.structure[i] = landmark;
  }

  return sfm_data;
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
