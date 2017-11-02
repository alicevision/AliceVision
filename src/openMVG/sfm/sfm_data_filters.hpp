
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_SFM_DATA_FILTERS_HPP
#define OPENMVG_SFM_DATA_FILTERS_HPP

#include "sfm_view.hpp"

#include "openMVG/stl/stl.hpp"
#include <openMVG/system/Logger.hpp>
#include <iterator>

namespace openMVG {
namespace sfm {

/// Filter a list of pair: Keep only the pair that are defined in index list
template <typename IterablePairs, typename IterableIndex>
static Pair_Set Pair_filter
(
  const IterablePairs & pairs,
  const IterableIndex & index
)
{
  Pair_Set kept_pairs;
  for (auto& it : pairs)
  {
    if (index.count(it.first) > 0 &&
      index.count(it.second) > 0)
    kept_pairs.insert(it);
  }
  return kept_pairs;
}

/// Remove observations with too large reprojection error.
/// Return the number of removed tracks.
static IndexT RemoveOutliers_PixelResidualError
(
  SfM_Data & sfm_data,
  const double dThresholdPixel,
  const unsigned int minTrackLength = 2
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
      const geometry::Pose3 pose = sfm_data.getPose(*view);
      const cameras::IntrinsicBase * intrinsic = sfm_data.intrinsics.at(view->getIntrinsicId()).get();
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

// Remove tracks that have a small angle (tracks with tiny angle leads to instable 3D points)
// Return the number of removed tracks
static IndexT RemoveOutliers_AngleError
(
  SfM_Data & sfm_data,
  const double dMinAcceptedAngle
)
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
      const geometry::Pose3 pose1 = sfm_data.getPose(*view1);
      const cameras::IntrinsicBase * intrinsic1 = sfm_data.intrinsics.at(view1->getIntrinsicId()).get();

      Observations::const_iterator itObs2 = itObs1;
      ++itObs2;
      for (; itObs2 != observations.end(); ++itObs2)
      {
        const View * view2 = sfm_data.views.at(itObs2->first).get();
        const geometry::Pose3 pose2 = sfm_data.getPose(*view2);
        const cameras::IntrinsicBase * intrinsic2 = sfm_data.intrinsics.at(view2->getIntrinsicId()).get();

        const double angle = AngleBetweenRay(
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

static bool eraseUnstablePoses(SfM_Data & sfm_data, const IndexT min_points_per_pose, std::set<IndexT>* outRemovedPosedId = NULL)
{
  IndexT removed_elements = 0;
  const Landmarks & landmarks = sfm_data.structure;

  // Count the observation poses occurrence
  Hash_Map<IndexT, IndexT> map_PoseId_Count; // TODO: add subpose
  // Init with 0 count (in order to be able to remove non referenced elements)
  for (Poses::const_iterator itPoses = sfm_data.GetPoses().begin();
    itPoses != sfm_data.GetPoses().end(); ++itPoses)
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
      const View * v = sfm_data.GetViews().at(ViewId).get();
      if (map_PoseId_Count.count(v->getPoseId()))
        map_PoseId_Count.at(v->getPoseId()) += 1;
      else
        map_PoseId_Count[v->getPoseId()] = 0;
    }
  }
  // If usage count is smaller than the threshold, remove the Pose
  for (Hash_Map<IndexT, IndexT>::const_iterator it = map_PoseId_Count.begin();
    it != map_PoseId_Count.end(); ++it)
  {
    if (it->second < min_points_per_pose)
    {
      sfm_data.erasePose(it->first);
      outRemovedPosedId->insert(it->first);
      ++removed_elements;
    }
  }
  if(removed_elements)
    OPENMVG_LOG_DEBUG("eraseUnstablePoses: " << removed_elements);
  return removed_elements > 0;
}

static bool eraseObservationsWithMissingPoses(SfM_Data & sfm_data, const IndexT min_points_per_landmark)
{
  IndexT removed_elements = 0;

  std::set<IndexT> reconstructedPoseIndexes;
  std::transform(sfm_data.GetPoses().begin(), sfm_data.GetPoses().end(),
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
      const View * v = sfm_data.GetViews().at(ViewId).get();
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
static bool eraseUnstablePosesAndObservations(
  SfM_Data & sfm_data,
  const IndexT min_points_per_pose = 6,
  const IndexT min_points_per_landmark = 2,
  std::set<IndexT>* outRemovedPosedId = NULL)
{
  IndexT remove_iteration = 0;
  bool bRemovedContent = false;
  do
  {
    bRemovedContent = false;
    if (eraseUnstablePoses(sfm_data, min_points_per_pose, outRemovedPosedId))
    {
      bRemovedContent = eraseObservationsWithMissingPoses(sfm_data, min_points_per_landmark);
      // Erase some observations can make some Poses index disappear so perform the process in a loop
    }
    remove_iteration += bRemovedContent ? 1 : 0;
  }
  while (bRemovedContent);

  return remove_iteration > 0;
}

} // namespace sfm
} // namespace openMVG

#endif // OPENMVG_SFM_DATA_FILTERS_HPP
