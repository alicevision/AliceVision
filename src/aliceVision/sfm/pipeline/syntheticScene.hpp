// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/sfm/sfm.hpp"
using namespace aliceVision;
using namespace aliceVision::sfm;

#include <random>
#include <iostream>


// Create from a synthetic scene (NViewDataSet) some SfM pipelines data provider:
//  - for contiguous triplets store the corresponding observations indexes

inline bool generateSyntheticMatches(
  matching::PairwiseMatches& pairwiseMatches,
  const NViewDataSet & synthetic_data,
  feature::EImageDescriberType descType)
{
  // For each view
  for (int j = 0; j < synthetic_data._n; ++j)
  {
    for (int jj = j+1; jj < j+3 ; ++jj)
    {
      for (int idx = 0; idx < synthetic_data._x[j].cols(); ++idx)
      {
        pairwiseMatches[Pair(j,(jj)%synthetic_data._n)][descType].emplace_back(idx,idx);
      }
    }
  }
  return true;
}


/// Compute the Root Mean Square Error of the residuals
static double RMSE(const SfMData & sfm_data)
{
  // Compute residuals for each observation
  std::vector<double> vec;
  for(Landmarks::const_iterator iterTracks = sfm_data.GetLandmarks().begin();
      iterTracks != sfm_data.GetLandmarks().end();
      ++iterTracks)
  {
    const Observations & obs = iterTracks->second.observations;
    for(Observations::const_iterator itObs = obs.begin();
      itObs != obs.end(); ++itObs)
    {
      const View * view = sfm_data.GetViews().find(itObs->first)->second.get();
      const geometry::Pose3 pose = sfm_data.getPose(*view);
      const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfm_data.GetIntrinsics().at(view->getIntrinsicId());
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);
      //ALICEVISION_LOG_DEBUG(residual);
      vec.push_back( residual(0) );
      vec.push_back( residual(1) );
    }
  }
  const Eigen::Map<Eigen::RowVectorXd> residuals(&vec[0], vec.size());
  const double RMSE = std::sqrt(residuals.squaredNorm() / vec.size());
  return RMSE;
}

// Translate a synthetic scene into a valid SfMData scene
// As only one intrinsic is defined we used shared intrinsic
SfMData getInputScene
(
  const NViewDataSet & d,
  const NViewDatasetConfigurator & config,
  camera::EINTRINSIC eintrinsic)
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
    sfm_data.views[i] = std::make_shared<View>
      ("", id_view, id_intrinsic, id_pose, config._cx *2, config._cy *2);
  }

  // 2. Poses
  for (int i = 0; i < nviews; ++i)
  {
    sfm_data.setPose(*sfm_data.views.at(i), geometry::Pose3(d._R[i], d._C[i]));
  }

  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx *2;
    const unsigned int h = config._cy *2;
    switch (eintrinsic)
    {
      case camera::PINHOLE_CAMERA:
        sfm_data.intrinsics[0] = std::make_shared<camera::Pinhole>
          (w, h, config._fx, config._cx, config._cy);
      break;
      case camera::PINHOLE_CAMERA_RADIAL1:
        sfm_data.intrinsics[0] = std::make_shared<camera::PinholeRadialK1>
          (w, h, config._fx, config._cx, config._cy, 0.0);
      break;
      case camera::PINHOLE_CAMERA_RADIAL3:
        sfm_data.intrinsics[0] = std::make_shared<camera::PinholeRadialK3>
          (w, h, config._fx, config._cx, config._cy, 0., 0., 0.);
      break;
      default:
        ALICEVISION_LOG_DEBUG("Not yet supported");
    }
  }

  // 4. Landmarks
  for (int i = 0; i < npoints; ++i) {
    // Collect the image of point i in each frame.
    Landmark landmark;
    landmark.X = d._X.col(i);
    for (int j = 0; j < nviews; ++j) {
      const Vec2 pt = d._x[j].col(i);
      landmark.observations[j] = Observation(pt, i);
    }
    sfm_data.structure[i] = landmark;
  }

  return sfm_data;
}

// Translate a synthetic scene into a valid SfMData scene
// As only one intrinsic is defined we used shared intrinsic
SfMData getInputRigScene(const NViewDataSet& d,
                          const NViewDatasetConfigurator& config,
                          camera::EINTRINSIC eintrinsic,
                          std::size_t nbSubposes,
                          std::size_t nbPosesPerCamera)
{
  // 1. Rig
  // 2. Views
  // 3. Poses
  // 4. Intrinsic data (shared, so only one camera intrinsic is defined)
  // 5. Landmarks

  // Translate the input dataset to a SfMData scene
  SfMData sfmData;

  const std::size_t nbPoses = d._C.size();
  const std::size_t nbPoints = d._X.cols();

  // 1. Rig

  const IndexT rigId = 0;
  sfmData.getRigs().emplace(rigId, Rig(nbSubposes));

  // 2. Views
  for (std::size_t cameraId = 0; cameraId < nbSubposes; ++cameraId)
  {
    for(std::size_t frameId = 0; frameId < nbPosesPerCamera; ++frameId)
    {
      const IndexT id_view = cameraId * nbPosesPerCamera + frameId;
      const IndexT id_pose = frameId;
      const IndexT id_intrinsic = 0; //(shared intrinsics)

      sfmData.views[id_view] = std::make_shared<View>("",
                                                 id_view,
                                                 id_intrinsic,
                                                 id_pose,
                                                 config._cx *2,
                                                 config._cy *2,
                                                 0,
                                                 cameraId);
    }
  }

  // 3. Poses
  for (int i = 0; i < nbPoses; ++i)
  {
    if((i < nbPosesPerCamera) || (i % nbPosesPerCamera == 0))
    {
      sfmData.setPose(*sfmData.views.at(i), geometry::Pose3(d._R[i], d._C[i]));
    }
  }

  // 4. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx *2;
    const unsigned int h = config._cy *2;
    switch (eintrinsic)
    {
      case camera::PINHOLE_CAMERA:
        sfmData.intrinsics[0] = std::make_shared<camera::Pinhole>
          (w, h, config._fx, config._cx, config._cy);
      break;
      case camera::PINHOLE_CAMERA_RADIAL1:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK1>
          (w, h, config._fx, config._cx, config._cy, 0.0);
      break;
      case camera::PINHOLE_CAMERA_RADIAL3:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK3>
          (w, h, config._fx, config._cx, config._cy, 0., 0., 0.);
      break;
      default:
        ALICEVISION_LOG_DEBUG("Not yet supported");
    }
  }

  // 5. Landmarks
  for (int i = 0; i < nbPoints; ++i) {
    // Collect the image of point i in each frame.
    Landmark landmark;
    landmark.X = d._X.col(i);
    for (int j = 0; j < nbPoses; ++j) {
      const Vec2 pt = d._x[j].col(i);
      landmark.observations[j] = Observation(pt, i);
    }
    sfmData.structure[i] = landmark;
  }

  return sfmData;
}
