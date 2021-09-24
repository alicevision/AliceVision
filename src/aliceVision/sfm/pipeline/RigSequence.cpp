// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RigSequence.hpp"
#include <aliceVision/stl/mapUtils.hpp>

#include <boost/functional/hash.hpp>

#include <algorithm>

namespace aliceVision {
namespace sfm {

using namespace sfmData;

IndexT getRigPoseId(IndexT rigId, IndexT frameId)
{
  std::size_t rigPoseId = static_cast<std::size_t>(rigId);
  boost::hash_combine(rigPoseId, static_cast<std::size_t>(frameId));

  return static_cast<IndexT>(rigPoseId);
}

double computeCameraScore(const SfMData& sfmData, const track::TracksPerView& tracksPerView, IndexT viewId)
{
  std::set<std::size_t> viewLandmarks;
  {
    // A. Compute 2D/3D matches
    // A1. list tracks ids used by the view
    const aliceVision::track::TrackIdSet& tracksIds = tracksPerView.at(static_cast<std::size_t>(viewId));

    // A2. intersects the track list with the reconstructed
    std::set<std::size_t> reconstructedTrackId;
    std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
                   std::inserter(reconstructedTrackId, reconstructedTrackId.begin()),
                   stl::RetrieveKey());

    // Get the ids of the already reconstructed tracks
    std::set_intersection(tracksIds.begin(), tracksIds.end(),
                          reconstructedTrackId.begin(),
                          reconstructedTrackId.end(),
                          std::inserter(viewLandmarks, viewLandmarks.begin()));

    if(viewLandmarks.empty())
    {
      // No match. The image has no connection with already reconstructed points.
      ALICEVISION_LOG_DEBUG("computeCameraScore: no reconstructed points");
      return 0.0;
    }
  }

  const sfmData::View& view = sfmData.getView(viewId);
  const geometry::Pose3 pose = sfmData.getPose(view).getTransform();
  const camera::IntrinsicBase * intrinsic = sfmData.intrinsics.at(view.getIntrinsicId()).get();

  double score = 0.0;

  for(auto landmarkId: viewLandmarks)
  {
    const Landmark& landmark = sfmData.getLandmarks().at(static_cast<IndexT>(landmarkId));


    sfmData::Observations::const_iterator itObs = landmark.observations.find(viewId);

    if(itObs != landmark.observations.end())
    {
      const Vec2 residual = intrinsic->residual(pose, landmark.X.homogeneous(), itObs->second.x);
      score += std::min(1.0 / residual.norm(), 4.0);
    }
  }
  return score;
}


void RigSequence::init(const track::TracksPerView& tracksPerView)
{
  for(const auto& viewPair : _sfmData.getViews())
  {
    const View& view = *(viewPair.second);

    if(view.isPartOfRig() && view.getRigId() == _rigId)
    {
      double score = 0.0;

      // compute pose score, sum of inverse reprojection errors
      if(_sfmData.isPoseAndIntrinsicDefined(view.getViewId()))
      {
        score = computeCameraScore(_sfmData, tracksPerView, view.getViewId());

        // add one to the number of poses for this rig relative sub-pose
        _rigInfoPerSubPose[view.getSubPoseId()].nbPose++;
      }

      // add the frame score
      _rigInfoPerFrame[view.getFrameId()][view.getSubPoseId()] = {view.getViewId(), view.isPoseIndependant(), score};
    }
  }

  // check if sub-pose are initialized
  for(auto& subPoseInfoPair : _rigInfoPerSubPose)
  {
    const IndexT subPoseId = subPoseInfoPair.first;
    SubPoseInfo& subPoseInfo = subPoseInfoPair.second;

    if(_sfmData.getRigs().at(_rigId).getSubPose(subPoseId).status != ERigSubPoseStatus::UNINITIALIZED)
      subPoseInfo.isInitialized = true;
  }

  ALICEVISION_LOG_INFO("Initialize rig calibration: "
                       << "\n\t- rig id: " << _rigId
                       << "\n\t- # detected sub-poses: " << _rigInfoPerSubPose.size()
                       << "\n\t- # detected frames: " << _rigInfoPerFrame.size());
}

void RigSequence::updateSfM(std::set<IndexT>& updatedViews)
{
  const Rig& rig = _sfmData.getRigs().at(_rigId);

  if(!rig.isFullyCalibrated())
  {
    if(!rig.isInitialized())
    {
      setupRelativePoses();
      rigResection(updatedViews);
    }
    setupRelativePoses();
  }
  rigResection(updatedViews);
}

void RigSequence::computeScores()
{
  const bool rigInitialized = _rig.isInitialized();

  for(auto& subPoseInfoPair : _rigInfoPerSubPose)
  {
    const IndexT subPoseId = subPoseInfoPair.first;
    SubPoseInfo& subPoseInfo = subPoseInfoPair.second;

    for(const auto& frameInfoPair : _rigInfoPerFrame)
    {
      const IndexT frameId = frameInfoPair.first;
      const RigFrame& rigFrame = frameInfoPair.second;

      const auto currentSubPoseIt = rigFrame.find(subPoseId);

      // skip current sub-pose
      if(currentSubPoseIt == rigFrame.end())
        continue;

      // compute frame score
      double frameScore = 0;
      for(const auto& otherPoseIt : rigFrame)
      {
        // if the rig is initialized, score relative to all non-independant cameras of the rig
        if(rigInitialized && !otherPoseIt.second.isPoseIndependant)
          continue;

        // score relative to other localized cameras of the rig
        if(otherPoseIt.first != subPoseId && otherPoseIt.second.score != 0.0)
          frameScore += currentSubPoseIt->second.score * otherPoseIt.second.score;
      }

      // keep best frame score
      if(frameScore > subPoseInfo.maxFrameScore)
      {
        subPoseInfo.maxFrameId = frameId;
        subPoseInfo.maxFrameScore = frameScore;
      }

      // compute total sub-pose score
      subPoseInfo.totalScore += frameScore;
    }
  }
}

void RigSequence::setupRelativePoses()
{
  const bool rigInitialized = _rig.isInitialized();

  // check if new subposes could be initialized (number of poses > N)
  std::vector<IndexT> subPosesToInitialized;
  {
    for(const auto& subPoseInfoPair : _rigInfoPerSubPose)
    {
      if(subPoseInfoPair.second.nbPose > _params.minNbCamerasForCalibration &&
         !subPoseInfoPair.second.isInitialized)
        subPosesToInitialized.emplace_back(subPoseInfoPair.first);
    }

    if(subPosesToInitialized.empty())
    {
      ALICEVISION_LOG_DEBUG("Cannot initialized a new sub-poses");
      return;
    }
  }

  // compute score for each camera (rig sub-pose) at each frame
  computeScores();

  // if the rig is uninitialized, keep only the best camera
  if(!rigInitialized && subPosesToInitialized.size() > 1)
  {
    IndexT bestSubPoseId = _rigInfoPerSubPose.begin()->first;
    SubPoseInfo& bestSubPoseInfo = _rigInfoPerSubPose.begin()->second;

    ALICEVISION_LOG_DEBUG("First sub-pose chosen:"
                          << "\n\t- rig id: "        << _rigId
                          << "\n\t- sub-pose id: "   << bestSubPoseId
                          << "\n\t- nbPoses: "       << bestSubPoseInfo.nbPose
                          << "\n\t- maxFrameId: "    << bestSubPoseInfo.maxFrameId
                          << "\n\t- maxFrameScore: " << bestSubPoseInfo.maxFrameScore);

    // select the best camera
    for(const auto& subPoseInfoPair :_rigInfoPerSubPose)
    {
      if(subPoseInfoPair.second.totalScore > bestSubPoseInfo.totalScore)
      {
        bestSubPoseId = subPoseInfoPair.first;
        bestSubPoseInfo = subPoseInfoPair.second;
      }
    }

    // keep only the best camera
    subPosesToInitialized.clear();
    subPosesToInitialized.push_back(bestSubPoseId);
  }

  // compute and add the new sub-pose(s)
  for(const IndexT subPoseId : subPosesToInitialized)
  {
    ALICEVISION_LOG_DEBUG("Add a new rig sub-pose."
                          << "\n\t- rig id: " << _rigId
                          << "\n\t- sub-pose id: " << subPoseId);

    SubPoseInfo& subPoseInfo = _rigInfoPerSubPose.at(subPoseId);

    const View& bestIndependantView = _sfmData.getView(_rigInfoPerFrame.at(subPoseInfo.maxFrameId).at(subPoseId).viewId);
    const geometry::Pose3& bestIndependantPose = _sfmData.getPoses().at(bestIndependantView.getPoseId()).getTransform();

    RigSubPose& newSubPose = _rig.getSubPose(subPoseId);
    newSubPose.status = ERigSubPoseStatus::ESTIMATED;

    subPoseInfo.isInitialized = true;

    if(rigInitialized)
    {
      const geometry::Pose3& rigPose = _sfmData.getPoses().at(getRigPoseId(_rigId, subPoseInfo.maxFrameId)).getTransform();
      newSubPose.pose = bestIndependantPose * rigPose.inverse();
    } // else pose is Identity
  }
}

void RigSequence::rigResection(std::set<IndexT>& updatedViews)
{;
  for(auto& frameInfoPair : _rigInfoPerFrame)
  {
    RigFrame& rigFrame = frameInfoPair.second;
    const IndexT frameId = frameInfoPair.first;
    const IndexT rigPoseId = getRigPoseId(_rigId, frameId);

    // if no rig pose, compute and add it
    if(_sfmData.getPoses().find(rigPoseId) == _sfmData.getPoses().end())
    {
      // TODO: use opengv resection for more than one valid view
      IndexT bestSubPoseId = UndefinedIndexT;
      double bestScore = 0.0;

      for(const auto& viewInfoPair: rigFrame)
      {
        const IndexT subPoseId = viewInfoPair.first;
        const ViewInfo& viewInfo = viewInfoPair.second;
        const auto itSubPoseInfoPair = _rigInfoPerSubPose.find(subPoseId);

        // Temporary: set the rig pose to the pose of the defined camera with the max score
        if(itSubPoseInfoPair != _rigInfoPerSubPose.end() &&
           itSubPoseInfoPair->second.isInitialized &&
           viewInfo.score > bestScore)
        {
          bestSubPoseId = subPoseId;
          bestScore = viewInfo.score;
        }
      }

      if(bestSubPoseId == UndefinedIndexT)
      {
        ALICEVISION_LOG_TRACE("No sub-pose localized, skip:\n\t- frame id: " << frameId);
        continue;
      }

      const RigSubPose& subPose = _rig.getSubPose(bestSubPoseId);
      const View& view = _sfmData.getView(rigFrame.at(bestSubPoseId).viewId);
      const IndexT independantPoseId = view.getPoseId();
      const CameraPose independantPose = _sfmData.getPoses().at(independantPoseId);
      const CameraPose rigPose(independantPose.getTransform() * subPose.pose.inverse());

      ALICEVISION_LOG_DEBUG("Add rig pose:"
                            << "\n\t- rig pose id: " << rigPoseId
                            << "\n\t- frame id: "    << frameId
                            << "\n\t- sub-pose id: " << bestSubPoseId);

      _sfmData.getPoses()[rigPoseId] = rigPose;
    }

    // remove independant poses and replace with rig pose
    for(auto& viewInfoPair: rigFrame)
    {
      const IndexT subPoseId = viewInfoPair.first;
      ViewInfo& viewInfo = viewInfoPair.second;
      const auto itSubPoseInfoPair = _rigInfoPerSubPose.find(subPoseId);

      if(!viewInfo.isPoseIndependant ||
         itSubPoseInfoPair == _rigInfoPerSubPose.end() ||
         !itSubPoseInfoPair->second.isInitialized)
        continue;

      View& view = _sfmData.getView(viewInfo.viewId);

      {
        Poses::iterator itPose = _sfmData.getPoses().find(view.getPoseId());

        if(itPose != _sfmData.getPoses().end())
        {
          _sfmData.getPoses().erase(itPose);
          ALICEVISION_LOG_TRACE("Erase independant pose:"
                                << "\n\t- pose id: "     << view.getPoseId()
                                << "\n\t- frame id: "    << frameId
                                << "\n\t- sub-pose id: " << subPoseId);
        }
      }

      view.setIndependantPose(false);
      view.setPoseId(rigPoseId);
      viewInfo.isPoseIndependant = false;
      updatedViews.insert(view.getViewId());
    }
  }
}

} // namespace sfm
} // namespace aliceVision
