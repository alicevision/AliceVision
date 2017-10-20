// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>
#include <aliceVision/sfm/sfm.hpp>

#include <random>
#include <iostream>

namespace aliceVision {
namespace sfm {

/**
 * @brief Create features from a known SfMData (synthetic scene).
 *
 * @param[out] out_featuresPerView
 * @param[in] sfmData synthetic SfM dataset
 * @param[in] descType
 * @param[in] noise
 */
template <typename NoiseGenerator>
void generateSyntheticFeatures(feature::FeaturesPerView& out_featuresPerView, feature::EImageDescriberType descType, const SfMData & sfmData, NoiseGenerator & noise)
{
  assert(descType != feature::EImageDescriberType::UNINITIALIZED);
  std::default_random_engine generator;

  // precompute output feature vectors size and resize
  {
    std::map<IndexT, std::size_t> nbFeatPerView;
    for(const auto& it: sfmData.GetViews())
    {
      nbFeatPerView[it.first] = 0;
    }
    for(const auto& it: sfmData.GetLandmarks())
    {
      const Landmark& landmark = it.second;

      for(const auto& obsIt: landmark.observations)
      {
        const IndexT viewId = obsIt.first;
        const Observation& obs = obsIt.second;
        nbFeatPerView[viewId] = std::max(nbFeatPerView[viewId], std::size_t(obs.id_feat+1));
      }
    }
    for(auto& it: nbFeatPerView)
    {
      // create Point Features vectors at the right size
      feature::PointFeatures pointFeatures(it.second);
      out_featuresPerView.addFeatures(it.first, descType, pointFeatures);
    }
  }
  // Fill with the observation values
  for(const auto& it: sfmData.GetLandmarks())
  {
    const Landmark& landmark = it.second;

    for(const auto& obsIt: landmark.observations)
    {
      const IndexT viewId = obsIt.first;
      const Observation& obs = obsIt.second;

      out_featuresPerView.getFeaturesPerDesc(viewId)[descType][obs.id_feat] = feature::PointFeature(obs.x(0) + noise(generator), obs.x(1) + noise(generator));
    }
  }
}

/**
 * Generate features matches between views from a known SfMData (synthetic scene).
 *
 * @param[out] out_pairwiseMatches
 * @param[in] sfmData synthetic SfM dataset
 * @param[in] descType
 */
inline void generateSyntheticMatches(
  matching::PairwiseMatches& out_pairwiseMatches,
  const SfMData & sfmData,
  feature::EImageDescriberType descType)
{

  for(const auto& it: sfmData.GetLandmarks())
  {
    const Landmark& landmark = it.second;
    const std::size_t limitMatches = std::min(std::size_t(3), landmark.observations.size());

    for(auto obsItI = landmark.observations.begin(); obsItI != landmark.observations.end(); ++obsItI)
    {
      const Observation& obsI = obsItI->second;
      // We don't need matches between all observations.
      // We will limit to matches between 3 observations of the same landmark.
      // At the end of the reconstruction process, they should be be fused again into one landmark.
      auto obsItJ = obsItI;
      for(std::size_t j = 1; j < limitMatches; ++j)
      {
        ++obsItJ;
        if(obsItJ == landmark.observations.end())
          obsItJ = landmark.observations.begin();

        const Observation& obsJ = obsItJ->second;

        out_pairwiseMatches[Pair(obsItI->first, obsItJ->first)][descType].emplace_back(obsItI->second.id_feat, obsItJ->second.id_feat);
      }
    }
  }
}


/// Compute the Root Mean Square Error of the residuals
inline double RMSE(const SfMData & sfm_data)
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
  for (int i = 0; i < npoints; ++i)
  {
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
inline SfMData getInputRigScene(const NViewDataSet& d,
                          const NViewDatasetConfigurator& config,
                          camera::EINTRINSIC eintrinsic)
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
  const std::size_t nbSubposes = 2;
  sfmData.getRigs().emplace(rigId, Rig(nbSubposes));
  Rig& rig = sfmData.getRigs().at(rigId);
  rig.getSubPose(0) = RigSubPose(geometry::Pose3(Mat3::Identity(), Vec3(-0.01, 0, 0)), ERigSubPoseStatus::CONSTANT);
  rig.getSubPose(1) = RigSubPose(geometry::Pose3(Mat3::Identity(), Vec3(+0.01, 0, 0)), ERigSubPoseStatus::CONSTANT);

  // 2. Views
  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subposeI = 0; subposeI < nbSubposes; ++subposeI)
    {
      const IndexT viewId = poseId * nbSubposes + subposeI;
      const IndexT intrinsicId = 0; //(shared intrinsics)

      sfmData.views[viewId] = std::make_shared<View>("",
                                                 viewId,
                                                 intrinsicId,
                                                 poseId,
                                                 config._cx *2,
                                                 config._cy *2,
                                                 rigId,
                                                 subposeI);
    }
  }
  const std::size_t nbViews = sfmData.views.size();

  // 3. Poses
  for (int poseId = 0; poseId < nbPoses; ++poseId)
  {
    sfmData.GetPoses()[poseId] = geometry::Pose3(d._R[poseId], d._C[poseId]);
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
      throw std::runtime_error("Intrinsic type is not implemented.");
    }
  }

  // 5. Landmarks
  for(int landmarkId = 0; landmarkId < nbPoints; ++landmarkId)
  {
    // Collect the image of point i in each frame.
    Landmark landmark;
    landmark.X = d._X.col(landmarkId);
    for(int viewId = 0; viewId < nbViews; ++viewId)
    {
      const View& view = *sfmData.views.at(viewId);
      geometry::Pose3 camPose = sfmData.getPose(view);
      const Vec2 pt = Project(sfmData.intrinsics.at(0)->get_projective_equivalent(camPose), landmark.X);
      landmark.observations[viewId] = Observation(pt, landmarkId);
    }
    sfmData.structure[landmarkId] = landmark;
  }

  return sfmData;
}

}
}
