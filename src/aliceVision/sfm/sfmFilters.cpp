// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmFilters.hpp"
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/system/Logger.hpp>

#include <iterator>

namespace aliceVision {
namespace sfm {

IndexT RemoveOutliers_PixelResidualError(sfmData::SfMData& sfmData,
                                         const double dThresholdPixel,
                                         const unsigned int minTrackLength)
{
  IndexT outlier_count = 0;
  sfmData::Landmarks::iterator iterTracks = sfmData.structure.begin();

  while(iterTracks != sfmData.structure.end())
  {
    sfmData::Observations & observations = iterTracks->second.observations;
    sfmData::Observations::iterator itObs = observations.begin();

    while(itObs != observations.end())
    {
      const sfmData::View * view = sfmData.views.at(itObs->first).get();
      const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
      const camera::IntrinsicBase * intrinsic = sfmData.intrinsics.at(view->getIntrinsicId()).get();
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);

      if((pose.depth(iterTracks->second.X) < 0) || (residual.norm() > dThresholdPixel))
      {
        ++outlier_count;
        itObs = observations.erase(itObs);
      }
      else
        ++itObs;
    }

    if (observations.empty() || observations.size() < minTrackLength)
      iterTracks = sfmData.structure.erase(iterTracks);
    else
      ++iterTracks;
  }
  return outlier_count;
}

IndexT RemoveOutliers_AngleError(sfmData::SfMData& sfmData, const double dMinAcceptedAngle)
{
  IndexT removedTrack_count = 0;
  sfmData::Landmarks::iterator iterTracks = sfmData.structure.begin();

  while(iterTracks != sfmData.structure.end())
  {
    sfmData::Observations & observations = iterTracks->second.observations;
    double max_angle = 0.0;
    for(sfmData::Observations::const_iterator itObs1 = observations.begin(); itObs1 != observations.end(); ++itObs1)
    {
      const sfmData::View * view1 = sfmData.views.at(itObs1->first).get();
      const geometry::Pose3 pose1 = sfmData.getPose(*view1).getTransform();
      const camera::IntrinsicBase * intrinsic1 = sfmData.intrinsics.at(view1->getIntrinsicId()).get();

      sfmData::Observations::const_iterator itObs2 = itObs1;
      ++itObs2;

      for(; itObs2 != observations.end(); ++itObs2)
      {
        const sfmData::View * view2 = sfmData.views.at(itObs2->first).get();
        const geometry::Pose3 pose2 = sfmData.getPose(*view2).getTransform();
        const camera::IntrinsicBase * intrinsic2 = sfmData.intrinsics.at(view2->getIntrinsicId()).get();

        const double angle = AngleBetweenRays(pose1, intrinsic1, pose2, intrinsic2, itObs1->second.x, itObs2->second.x);
        max_angle = std::max(angle, max_angle);
      }
    }
    if (max_angle < dMinAcceptedAngle)
    {
      iterTracks = sfmData.structure.erase(iterTracks);
      ++removedTrack_count;
    }
    else
      ++iterTracks;
  }
  return removedTrack_count;
}

bool eraseUnstablePoses(sfmData::SfMData& sfmData, const IndexT min_points_per_pose, std::set<IndexT>* outRemovedViewsId)
{
  IndexT removed_elements = 0;
  const sfmData::Landmarks & landmarks = sfmData.structure;

  // Count the observation poses occurrence
  HashMap<IndexT, IndexT> posesCount;

  // Init with 0 count, undefined rig id (in order to be able to remove non referenced elements)
  for(sfmData::Poses::const_iterator itPoses = sfmData.getPoses().begin(); itPoses != sfmData.getPoses().end(); ++itPoses)
    posesCount[itPoses->first] = 0;

  // Count occurrence of the poses in the Landmark observations
  for(sfmData::Landmarks::const_iterator itLandmarks = landmarks.begin(); itLandmarks != landmarks.end(); ++itLandmarks)
  {
    const sfmData::Observations & observations = itLandmarks->second.observations;
    for(sfmData::Observations::const_iterator itObs = observations.begin(); itObs != observations.end(); ++itObs)
    {
      const IndexT viewId = itObs->first;
      const sfmData::View * v = sfmData.getViews().at(viewId).get();
      const auto poseInfoIt = posesCount.find(v->getPoseId());

      if(poseInfoIt != posesCount.end())
        poseInfoIt->second++;
      else // all pose should be defined in map_PoseId_Count
        throw std::runtime_error(std::string("eraseUnstablePoses: found unknown pose id referenced by a view.\n\t- view id: ")
                                 + std::to_string(v->getViewId()) + std::string("\n\t- pose id: ") + std::to_string(v->getPoseId()));
    }
  }

  // If usage count is smaller than the threshold, remove the Pose
  for(HashMap<IndexT, IndexT>::const_iterator it = posesCount.begin(); it != posesCount.end(); ++it)
  {
    if(it->second < min_points_per_pose)
    {
      sfmData.erasePose(it->first, true); // no throw

      for(auto& viewPair : sfmData.getViews())
      {
        if(viewPair.second->getPoseId() == it->first)
        {
          if(viewPair.second->isPartOfRig())
          {
            // the pose is now independant
            viewPair.second->setPoseId(viewPair.first);
            viewPair.second->setIndependantPose(true);
          }

          // add view id to the removedViewsId set
          if(outRemovedViewsId != NULL)
            outRemovedViewsId->insert(viewPair.first);
        }
      }
      ++removed_elements;
    }
  }
  if(removed_elements)
    ALICEVISION_LOG_DEBUG("eraseUnstablePoses: " << removed_elements);
  return removed_elements > 0;
}

bool eraseObservationsWithMissingPoses(sfmData::SfMData& sfmData, const IndexT min_points_per_landmark)
{
  IndexT removed_elements = 0;

  std::set<IndexT> reconstructedPoseIndexes;
  std::transform(sfmData.getPoses().begin(), sfmData.getPoses().end(),
    std::inserter(reconstructedPoseIndexes, reconstructedPoseIndexes.begin()), stl::RetrieveKey());

  // For each landmark:
  //  - Check if we need to keep the observations & the track
  sfmData::Landmarks::iterator itLandmarks = sfmData.structure.begin();

  while(itLandmarks != sfmData.structure.end())
  {
    sfmData::Observations& observations = itLandmarks->second.observations;
    sfmData::Observations::iterator itObs = observations.begin();

    while (itObs != observations.end())
    {
      const IndexT ViewId = itObs->first;
      const sfmData::View* v = sfmData.getViews().at(ViewId).get();
      if(reconstructedPoseIndexes.count(v->getPoseId()) == 0)
      {
        itObs = observations.erase(itObs);
        ++removed_elements;
      }
      else
        ++itObs;
    }

    if(observations.empty() || observations.size() < min_points_per_landmark)
      itLandmarks = sfmData.structure.erase(itLandmarks);
    else
      ++itLandmarks;
  }
  return removed_elements > 0;
}

/// Remove unstable content from analysis of the sfm_data structure
bool eraseUnstablePosesAndObservations(sfmData::SfMData& sfmData,
                                       const IndexT min_points_per_pose,
                                       const IndexT min_points_per_landmark,
                                       std::set<IndexT>* outRemovedViewsId)
{
  IndexT removeIteration = 0;
  bool removedContent = false;
  bool removedPoses = false;
  bool removedObservations = false;
  do
  {
    removedContent = false;
    if(eraseUnstablePoses(sfmData, min_points_per_pose, outRemovedViewsId))
    {
      removedPoses = true;
      removedContent = eraseObservationsWithMissingPoses(sfmData, min_points_per_landmark);
      if(removedContent)
        removedObservations = true;
      // Erase some observations can make some Poses index disappear so perform the process in a loop
    }
    removeIteration += removedContent ? 1 : 0;
  }
  while(removedContent);

  return removedPoses || removedObservations;
}

} // namespace sfm
} // namespace aliceVision
