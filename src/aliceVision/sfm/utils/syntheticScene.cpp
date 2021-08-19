// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "syntheticScene.hpp"
#include <aliceVision/sfm/sfm.hpp>

#include <random>
#include <iostream>

namespace aliceVision {
namespace sfm {

void generateSyntheticMatches(matching::PairwiseMatches& out_pairwiseMatches,
                              const sfmData::SfMData& sfmData,
                              feature::EImageDescriberType descType)
{
  for(const auto& it: sfmData.getLandmarks())
  {
    const sfmData::Landmark& landmark = it.second;
    const std::size_t limitMatches = std::min(std::size_t(3), landmark.observations.size());

    for(auto obsItI = landmark.observations.begin(); obsItI != landmark.observations.end(); ++obsItI)
    {
      const sfmData::Observation& obsI = obsItI->second;
      // We don't need matches between all observations.
      // We will limit to matches between 3 observations of the same landmark.
      // At the end of the reconstruction process, they should be be fused again into one landmark.
      auto obsItJ = obsItI;
      for(std::size_t j = 1; j < limitMatches; ++j)
      {
        ++obsItJ;
        if(obsItJ == landmark.observations.end())
          obsItJ = landmark.observations.begin();

        const sfmData::Observation& obsJ = obsItJ->second;

        out_pairwiseMatches[Pair(obsItI->first, obsItJ->first)][descType].emplace_back(obsItI->second.id_feat, obsItJ->second.id_feat);
      }
    }
  }
}

sfmData::SfMData getInputScene(const NViewDataSet& d,
                               const NViewDatasetConfigurator& config,
                               camera::EINTRINSIC eintrinsic)
{
  // Translate the input dataset to a SfMData scene
  sfmData::SfMData sfmData;

  // 1. Views
  // 2. Poses
  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  // 4. Landmarks

  const int nviews = d._C.size();
  const int npoints = d._X.cols();

  // 1. Views
  for(int i = 0; i < nviews; ++i)
  {
    const IndexT viewId = i, poseId = i, intrinsicId = 0; //(shared intrinsics)
    sfmData.views[i] = std::make_shared<sfmData::View>("", viewId, intrinsicId, poseId, config._cx * 2, config._cy * 2);
  }

  // 2. Poses
  for(int i = 0; i < nviews; ++i)
  {
    sfmData.setPose(*sfmData.views.at(i), sfmData::CameraPose(geometry::Pose3(d._R[i], d._C[i])));
  }

  // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx *2;
    const unsigned int h = config._cy *2;
    switch (eintrinsic)
    {
        case camera::EINTRINSIC::PINHOLE_CAMERA:
        sfmData.intrinsics[0] = std::make_shared<camera::Pinhole>
          (w, h, config._fx, config._fx, 0, 0);
      break;
        case camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK1>
          (w, h, config._fx, config._fx, 0, 0, 0.0);
      break;
        case camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK3>
          (w, h, config._fx, config._fx, 0, 0, 0., 0., 0.);
      break;
      default:
        ALICEVISION_LOG_DEBUG("Not yet supported");
    }
  }

  // 4. Landmarks
  const double unknownScale = 0.0;
  for(int i = 0; i < npoints; ++i)
  {
    // Collect the image of point i in each frame.
    sfmData::Landmark landmark;
    landmark.X = d._X.col(i);
    for (int j = 0; j < nviews; ++j) {
      const Vec2 pt = d._x[j].col(i);
      landmark.observations[j] = sfmData::Observation(pt, i, unknownScale);
    }
    sfmData.structure[i] = landmark;
  }

  return sfmData;
}

sfmData::SfMData getInputRigScene(const NViewDataSet& d,
                                  const NViewDatasetConfigurator& config,
                                  camera::EINTRINSIC eintrinsic)
{
  // 1. Rig
  // 2. Views
  // 3. Poses
  // 4. Intrinsic data (shared, so only one camera intrinsic is defined)
  // 5. Landmarks

  // Translate the input dataset to a SfMData scene
  sfmData::SfMData sfmData;

  const std::size_t nbPoses = d._C.size();
  const std::size_t nbPoints = d._X.cols();

  // 1. Rig
  const IndexT rigId = 0;
  const std::size_t nbSubposes = 2;
  sfmData.getRigs().emplace(rigId, sfmData::Rig(nbSubposes));
  sfmData::Rig& rig = sfmData.getRigs().at(rigId);
  rig.getSubPose(0) = sfmData::RigSubPose(geometry::Pose3(Mat3::Identity(), Vec3(-0.01, 0, 0)), sfmData::ERigSubPoseStatus::CONSTANT);
  rig.getSubPose(1) = sfmData::RigSubPose(geometry::Pose3(Mat3::Identity(), Vec3(+0.01, 0, 0)), sfmData::ERigSubPoseStatus::CONSTANT);

  // 2. Views
  for(std::size_t poseId = 0; poseId < nbPoses; ++poseId)
  {
    for(std::size_t subposeI = 0; subposeI < nbSubposes; ++subposeI)
    {
      const IndexT viewId = poseId * nbSubposes + subposeI;
      const IndexT intrinsicId = 0; //(shared intrinsics)

      auto viewPtr = std::make_shared<sfmData::View>("", viewId, intrinsicId, poseId, config._cx * 2, config._cy * 2, rigId, subposeI);
      viewPtr->setFrameId(poseId);
      viewPtr->setIndependantPose(false);
      sfmData.views[viewId] = viewPtr;
    }
  }
  const std::size_t nbViews = sfmData.views.size();

  // 3. Poses
  for(int poseId = 0; poseId < nbPoses; ++poseId)
  {
    sfmData.setAbsolutePose(static_cast<IndexT>(poseId), sfmData::CameraPose(geometry::Pose3(d._R[poseId], d._C[poseId])));
  }

  // 4. Intrinsic data (shared, so only one camera intrinsic is defined)
  {
    const unsigned int w = config._cx * 2;
    const unsigned int h = config._cy * 2;
    switch (eintrinsic)
    {
      case camera::EINTRINSIC::PINHOLE_CAMERA:
        sfmData.intrinsics[0] = std::make_shared<camera::Pinhole>(w, h, config._fx, config._fx, 0, 0);
      break;
      case camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK1>(w, h, config._fx, config._fx, 0, 0, 0.0);
      break;
      case camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
        sfmData.intrinsics[0] = std::make_shared<camera::PinholeRadialK3>(w, h, config._fx, config._fx, 0, 0, 0., 0., 0.);
      break;
      default:
      throw std::runtime_error("Intrinsic type is not implemented.");
    }
  }

  // 5. Landmarks
  const double unknownScale = 0.0;
  for(int landmarkId = 0; landmarkId < nbPoints; ++landmarkId)
  {
    // Collect the image of point i in each frame.
    sfmData::Landmark landmark;
    landmark.X = d._X.col(landmarkId);
    for(int viewId = 0; viewId < nbViews; ++viewId)
    {
      const sfmData::View& view = *sfmData.views.at(viewId);
      const geometry::Pose3 camPose = sfmData.getPose(view).getTransform();

      std::shared_ptr<camera::IntrinsicBase> cam = sfmData.intrinsics.at(0);
      std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
      if (!camPinHole) {
        ALICEVISION_LOG_ERROR("Camera is not pinhole in getInputRigScene");
        continue;
      }

      const Vec2 pt = project(camPinHole->getProjectiveEquivalent(camPose), landmark.X);
      landmark.observations[viewId] = sfmData::Observation(pt, landmarkId, unknownScale);
    }
    sfmData.structure[landmarkId] = landmark;
  }

  return sfmData;
}

} // namespace sfm
} // namespace aliceVision
