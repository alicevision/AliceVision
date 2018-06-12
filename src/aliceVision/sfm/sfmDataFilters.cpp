// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmDataFilters.hpp"
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/system/Logger.hpp>

#include <iterator>

namespace aliceVision {
namespace sfm {

IndexT RemoveOutliers_PixelResidualError
(
  SfMData & sfm_data,
  const double dThresholdPixel,
  const unsigned int minTrackLength
)
{
  IndexT outlier_count = 0;
  Landmarks::iterator iterTracks = sfm_data.structure.begin();
  while (iterTracks != sfm_data.structure.end())
  {
    Observations & observations = iterTracks->second.observations;
    Observations::iterator itObs = observations.begin();
    while (itObs != observations.end())
    {
      const View * view = sfm_data.views.at(itObs->first).get();
      const geometry::Pose3 pose = sfm_data.getPose(*view).getTransform();
      const camera::IntrinsicBase * intrinsic = sfm_data.intrinsics.at(view->getIntrinsicId()).get();
      const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X, itObs->second.x);
      if((pose.depth(iterTracks->second.X) < 0) ||
         (residual.norm() > dThresholdPixel))
      {
        ++outlier_count;
        itObs = observations.erase(itObs);
      }
      else
        ++itObs;
    }
    if (observations.empty() || observations.size() < minTrackLength)
      iterTracks = sfm_data.structure.erase(iterTracks);
    else
      ++iterTracks;
  }
  return outlier_count;
}

IndexT RemoveOutliers_AngleError(SfMData& sfm_data, const double dMinAcceptedAngle)
{
  IndexT removedTrack_count = 0;
  Landmarks::iterator iterTracks = sfm_data.structure.begin();
  while (iterTracks != sfm_data.structure.end())
  {
    Observations & observations = iterTracks->second.observations;
    double max_angle = 0.0;
    for (Observations::const_iterator itObs1 = observations.begin();
      itObs1 != observations.end(); ++itObs1)
    {
      const View * view1 = sfm_data.views.at(itObs1->first).get();
      const geometry::Pose3 pose1 = sfm_data.getPose(*view1).getTransform();
      const camera::IntrinsicBase * intrinsic1 = sfm_data.intrinsics.at(view1->getIntrinsicId()).get();

      Observations::const_iterator itObs2 = itObs1;
      ++itObs2;
      for (; itObs2 != observations.end(); ++itObs2)
      {
        const View * view2 = sfm_data.views.at(itObs2->first).get();
        const geometry::Pose3 pose2 = sfm_data.getPose(*view2).getTransform();
        const camera::IntrinsicBase * intrinsic2 = sfm_data.intrinsics.at(view2->getIntrinsicId()).get();

        const double angle = AngleBetweenRays(
          pose1, intrinsic1, pose2, intrinsic2,
          itObs1->second.x, itObs2->second.x);
        max_angle = std::max(angle, max_angle);
      }
    }
    if (max_angle < dMinAcceptedAngle)
    {
      iterTracks = sfm_data.structure.erase(iterTracks);
      ++removedTrack_count;
    }
    else
      ++iterTracks;
  }
  return removedTrack_count;
}

bool eraseUnstablePoses(SfMData& sfm_data, const IndexT min_points_per_pose, std::set<IndexT>* outRemovedPosedId)
{
  IndexT removed_elements = 0;
  const Landmarks & landmarks = sfm_data.structure;

  // Count the observation poses occurrence
  HashMap<IndexT, IndexT> map_PoseId_Count; // TODO: add subpose
  // Init with 0 count (in order to be able to remove non referenced elements)
  for (Poses::const_iterator itPoses = sfm_data.getPoses().begin();
    itPoses != sfm_data.getPoses().end(); ++itPoses)
  {
    map_PoseId_Count[itPoses->first] = 0;
  }

  // Count occurrence of the poses in the Landmark observations
  for (Landmarks::const_iterator itLandmarks = landmarks.begin();
    itLandmarks != landmarks.end(); ++itLandmarks)
  {
    const Observations & observations = itLandmarks->second.observations;
    for (Observations::const_iterator itObs = observations.begin();
      itObs != observations.end(); ++itObs)
    {
      const IndexT ViewId = itObs->first;
      const View * v = sfm_data.getViews().at(ViewId).get();
      if (map_PoseId_Count.count(v->getPoseId()))
        map_PoseId_Count.at(v->getPoseId()) += 1;
      else
        map_PoseId_Count[v->getPoseId()] = 0;
    }
  }
  // If usage count is smaller than the threshold, remove the Pose
  for (HashMap<IndexT, IndexT>::const_iterator it = map_PoseId_Count.begin();
    it != map_PoseId_Count.end(); ++it)
  {
    if (it->second < min_points_per_pose)
    {
      sfm_data.erasePose(it->first);
      if (outRemovedPosedId != NULL)
        outRemovedPosedId->insert(it->first);
      ++removed_elements;
    }
  }
  if(removed_elements)
    ALICEVISION_LOG_DEBUG("eraseUnstablePoses: " << removed_elements);
  return removed_elements > 0;
}

bool eraseObservationsWithMissingPoses(SfMData& sfm_data, const IndexT min_points_per_landmark)
{
  IndexT removed_elements = 0;

  std::set<IndexT> reconstructedPoseIndexes;
  std::transform(sfm_data.getPoses().begin(), sfm_data.getPoses().end(),
    std::inserter(reconstructedPoseIndexes, reconstructedPoseIndexes.begin()), stl::RetrieveKey());

  // For each landmark:
  //  - Check if we need to keep the observations & the track
  Landmarks::iterator itLandmarks = sfm_data.structure.begin();
  while (itLandmarks != sfm_data.structure.end())
  {
    Observations & observations = itLandmarks->second.observations;
    Observations::iterator itObs = observations.begin();
    while (itObs != observations.end())
    {
      const IndexT ViewId = itObs->first;
      const View * v = sfm_data.getViews().at(ViewId).get();
      if (reconstructedPoseIndexes.count(v->getPoseId()) == 0)
      {
        itObs = observations.erase(itObs);
        ++removed_elements;
      }
      else
        ++itObs;
    }
    if (observations.empty() || observations.size() < min_points_per_landmark)
      itLandmarks = sfm_data.structure.erase(itLandmarks);
    else
      ++itLandmarks;
  }
  return removed_elements > 0;
}

/// Remove unstable content from analysis of the sfm_data structure
bool eraseUnstablePosesAndObservations(
  SfMData& sfm_data,
  const IndexT min_points_per_pose,
  const IndexT min_points_per_landmark,
  std::set<IndexT>* outRemovedPosedId)
{
  IndexT remove_iteration = 0;
  bool bRemovedContent = false;
  bool bRemovedPoses = false;
  bool bRemovedObservations = false;
  do
  {
    bRemovedContent = false;
    if (eraseUnstablePoses(sfm_data, min_points_per_pose, outRemovedPosedId))
    {
      bRemovedPoses = true;
      bRemovedContent = eraseObservationsWithMissingPoses(sfm_data, min_points_per_landmark);
      if (bRemovedContent)
        bRemovedObservations = true;
      // Erase some observations can make some Poses index disappear so perform the process in a loop
    }
    remove_iteration += bRemovedContent ? 1 : 0;
  }
  while (bRemovedContent);

  return bRemovedPoses || bRemovedObservations;
}

} // namespace sfm
} // namespace aliceVision
