// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>

#include <cmath>
#include <cstdio>
#include <iostream>

#define BOOST_TEST_MODULE bundleAdjustment

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;

double RMSE(const SfMData& sfmData);

SfMData getInputScene(const NViewDataSet& d, const NViewDatasetConfigurator& config, EINTRINSIC eintrinsic);

track::TracksPerView getTracksPerViews(const SfMData& sfmData);

// Test summary:
// - Create a SfMData scene from a synthetic dataset
//   - since random noise have been added on 2d data point (initial residual is not small)
// - Check that residual is small once the generic Bundle Adjustment framework have been called.
// - Perform the test for all the plausible intrinsic camera models

BOOST_AUTO_TEST_CASE(BUNDLE_ADJUSTMENT_EffectiveMinimization_Pinhole)
{

  const int nviews = 3;
  const int npoints = 6;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);

  const double dResidual_before = RMSE(sfmData);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  BOOST_CHECK( ba_object->adjust(sfmData) );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

BOOST_AUTO_TEST_CASE(BUNDLE_ADJUSTMENT_EffectiveMinimization_PinholeRadialK1)
{
  const int nviews = 3;
  const int npoints = 6;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA_RADIAL1);

  const double dResidual_before = RMSE(sfmData);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  BOOST_CHECK( ba_object->adjust(sfmData) );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

BOOST_AUTO_TEST_CASE(BUNDLE_ADJUSTMENT_EffectiveMinimization_PinholeRadialK3)
{
  const int nviews = 3;
  const int npoints = 6;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA_RADIAL3);

  const double dResidual_before = RMSE(sfmData);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  BOOST_CHECK( ba_object->adjust(sfmData) );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

BOOST_AUTO_TEST_CASE(BUNDLE_ADJUSTMENT_EffectiveMinimization_PinholeBrownT2)
{
  const int nviews = 3;
  const int npoints = 6;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA_BROWN);

  const double dResidual_before = RMSE(sfmData);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  BOOST_CHECK( ba_object->adjust(sfmData) );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

BOOST_AUTO_TEST_CASE(BUNDLE_ADJUSTMENT_EffectiveMinimization_PinholeFisheye)
{
  const int nviews = 3;
  const int npoints = 6;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA_FISHEYE);

  const double dResidual_before = RMSE(sfmData);

  // Call the BA interface and let it refine (Structure and Camera parameters [Intrinsics|Motion])
  std::shared_ptr<BundleAdjustment> ba_object = std::make_shared<BundleAdjustmentCeres>();
  BOOST_CHECK( ba_object->adjust(sfmData) );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

BOOST_AUTO_TEST_CASE(LOCAL_BUNDLE_ADJUSTMENT_EffectiveMinimization_Pinhole_CamerasRing)
{
  const int nviews = 4;
  const int npoints = 3;
  const NViewDatasetConfigurator config;
  const NViewDataSet d = NRealisticCamerasRing(nviews, npoints, config);

  // Translate the input dataset to a SfMData scene
  SfMData sfmData = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA);
  SfMData sfmData_notRefined = getInputScene(d, config, EINTRINSIC::PINHOLE_CAMERA); // used to compate which parameters are refined.

  // Transform the views scheme
  //          v0 - v3
  //  from    |  X  |     =>  where each view sees all the 3D points p0, p1 and p2
  //          v1 - v2
  //
  //  to      v0   v3             p0   p1   p2
  //          |     |     =>     /  \ /  \ /  \
  //          v1 - v2           v0   v1   v2   v3
  // removing adequate observations:
  sfmData.getLandmarks().at(0).observations.erase(2);
  sfmData.getLandmarks().at(0).observations.erase(3);
  sfmData.getLandmarks().at(1).observations.erase(0);
  sfmData.getLandmarks().at(1).observations.erase(3);
  sfmData.getLandmarks().at(2).observations.erase(0);
  sfmData.getLandmarks().at(2).observations.erase(1);

  // lock common intrinsic
  // if it's not locked, all views will have a distance of 1 as all views share a common intrinsic.
  sfmData.getIntrinsics().begin()->second->lock();

  track::TracksPerView tracksPerView = getTracksPerViews(sfmData);

  // Set the view "v0' as new (graph-distance(v0) = 0):
  std::set<IndexT> newReconstructedViews;
  newReconstructedViews.insert(0);

  const double dResidual_before = RMSE(sfmData);

  // Call the Local BA interface and let it refine
  BundleAdjustmentCeres::CeresOptions options;
  options.setDenseBA();

  std::shared_ptr<LocalBundleAdjustmentGraph> localBAGraph = std::make_shared<LocalBundleAdjustmentGraph>(sfmData);
  localBAGraph->setGraphDistanceLimit(1); // the default value is '1'

  /* DETAILS: 
   * With the previous reconstruction scheme & parameters:
   *  -- Graph-distances:
   *    dist(v0) == 0 [because it is set to New]
   *    dist(v1) == 1 [because it shares 'p0' with v0]
   *    dist(v2) == 2 [because it shares 'p1' with v1]
   *    dist(v3) == 3 [because it shares 'p2' with v2]
   *  -- Local BA state: (due to the graph-distance)
   *    state(v0) = refined  [because its dist. is <= kLimitDistance]
   *    state(v1) = refined  [because its dist. is <= kLimitDistance]
   *    state(v2) = constant [because its dist. is == kLimitDistance + 1]
   *    state(v3) = ignored  [because its dist. is > kLimitDistance]
   *    state(p0) = refined  [because it is seen by at least one refined view (v0 & v1)]
   *    state(p1) = refined  [because it is seen by at least one refined view (v1)]
   *    state(p2) = ignored  [because it is not seen by any refined view]
   */
   
  // Assign the refinement rule for all the parameters (poses, landmarks & intrinsics) according to the LBA strategy:
  // 1. Add the new reconstructed views to the graph
  const std::size_t kMinNbOfMatches = 1;
  localBAGraph->updateGraphWithNewViews(sfmData, tracksPerView, newReconstructedViews, kMinNbOfMatches);
  // 2. Compute the graph-distance between each newly reconstructed views and all the reconstructed views
  localBAGraph->computeGraphDistances(sfmData, newReconstructedViews);
  // 3. Use the graph-distances to assign a LBA state (Refine, Constant & Ignore) for each parameter (poses, intrinsics & landmarks)
  localBAGraph->convertDistancesToStates(sfmData);

  BOOST_CHECK_EQUAL(localBAGraph->countNodes(), 4); // 4 views => 4 nodes
  BOOST_CHECK_EQUAL(localBAGraph->countEdges(), 6); // landmarks connections: 6 edges created (see scheme)

  BOOST_CHECK_EQUAL(localBAGraph->getNbPosesPerState(BundleAdjustment::EParameterState::REFINED), 2);     // v0 & v1
  BOOST_CHECK_EQUAL(localBAGraph->getNbPosesPerState(BundleAdjustment::EParameterState::CONSTANT), 1);    // v2
  BOOST_CHECK_EQUAL(localBAGraph->getNbPosesPerState(BundleAdjustment::EParameterState::IGNORED), 1);     // v3
  BOOST_CHECK_EQUAL(localBAGraph->getNbLandmarksPerState(BundleAdjustment::EParameterState::REFINED), 2); // p0 & p1
  BOOST_CHECK_EQUAL(localBAGraph->getNbLandmarksPerState(BundleAdjustment::EParameterState::CONSTANT), 0);
  BOOST_CHECK_EQUAL(localBAGraph->getNbLandmarksPerState(BundleAdjustment::EParameterState::IGNORED), 1); // p2

  std::shared_ptr<BundleAdjustmentCeres> BA = std::make_shared<BundleAdjustmentCeres>(options);
  BA->useLocalStrategyGraph(localBAGraph);
  BOOST_CHECK( BA->useLocalStrategy() );
  BOOST_CHECK( BA->adjust(sfmData) );

  // Check views:
  BOOST_CHECK( !(sfmData.getPose(*sfmData.views[0].get()) == sfmData_notRefined.getPose(*sfmData_notRefined.views[0].get())) ); // v0 refined
  BOOST_CHECK( !(sfmData.getPose(*sfmData.views[1].get()) == sfmData_notRefined.getPose(*sfmData_notRefined.views[1].get())) ); // v1 refined
  BOOST_CHECK( sfmData.getPose(*sfmData.views[2].get()) == sfmData_notRefined.getPose(*sfmData_notRefined.views[2].get()) ); // v2 constant
  BOOST_CHECK( sfmData.getPose(*sfmData.views[2].get()) == sfmData_notRefined.getPose(*sfmData_notRefined.views[2].get()) ); // v2 ignored

  // Check 3D points
  BOOST_CHECK( sfmData.structure[0].X != sfmData_notRefined.structure[0].X ); // p0 refined
  BOOST_CHECK( sfmData.structure[1].X != sfmData_notRefined.structure[1].X ); // p1 refined
  BOOST_CHECK_EQUAL( sfmData.structure[2].X, sfmData_notRefined.structure[2].X ); // p2 ignored

  // Not refined parameters:
  BOOST_CHECK( sfmData.structure[2].X == sfmData_notRefined.structure[2].X );

  const double dResidual_after = RMSE(sfmData);
  BOOST_CHECK_LT(dResidual_after, dResidual_before);
}

/// Compute the Root Mean Square Error of the residuals
double RMSE(const SfMData & sfm_data)
{
  // Compute residuals for each observation
  std::vector<double> vec;
  for(Landmarks::const_iterator iterTracks = sfm_data.getLandmarks().begin();
      iterTracks != sfm_data.getLandmarks().end();
      ++iterTracks)
  {
    const Observations & observations = iterTracks->second.observations;
    for(Observations::const_iterator itObs = observations.begin();
      itObs != observations.end(); ++itObs)
    {
      const View* view = sfm_data.getViews().find(itObs->first)->second.get();
      const Pose3 pose = sfm_data.getPose(*view).getTransform();
      const std::shared_ptr<IntrinsicBase> intrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId())->second;
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X.homogeneous(), itObs->second.x);
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
SfMData getInputScene(const NViewDataSet & d, const NViewDatasetConfigurator & config, EINTRINSIC eintrinsic)
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
    sfm_data.setPose(*sfm_data.views.at(i), CameraPose(pose));
  }

  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx *2;
    const unsigned int h = config._cy *2;
    sfm_data.intrinsics[0] = createIntrinsic(eintrinsic, w, h, config._fx, config._fx, 0, 0);
  }

  // 4. Landmarks
  const double unknownScale = 0.0;
  for (int i = 0; i < npoints; ++i) {

    // Collect the image of point i in each frame.
    Landmark landmark;
    landmark.X = d._X.col(i);
    for (int j = 0; j < nviews; ++j) {
      Vec2 pt = d._x[j].col(i);
      // => random noise between [-.5,.5] is added
      pt(0) += rand()/RAND_MAX - .5;
      pt(1) += rand()/RAND_MAX - .5;

      landmark.observations[j] = Observation(pt, i, unknownScale);
    }
    sfm_data.structure[i] = landmark;
  }

  return sfm_data;
}

track::TracksPerView getTracksPerViews(const SfMData& sfmData)
{
  track::TracksPerView tracksPerView;
  for (const auto& landIt : sfmData.getLandmarks())
  {
    for (const auto& obsIt : landIt.second.observations)
    {
      IndexT viewId = obsIt.first;
      track::TrackIdSet& tracksSet = tracksPerView[viewId];
      tracksSet.push_back(landIt.first);
    }
  }

  // sort tracks Ids in each view
  #pragma omp parallel for
  for(int i = 0; i < tracksPerView.size(); ++i)
  {
    track::TracksPerView::iterator it = tracksPerView.begin();
    std::advance(it, i);
    std::sort(it->second.begin(), it->second.end());
  }

  return tracksPerView;
}

