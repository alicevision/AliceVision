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
#include <aliceVision/sfm/BundleAdjustment.hpp>

#include <iterator>

namespace aliceVision {
namespace sfm {

IndexT RemoveOutliers_PixelResidualError(sfmData::SfMData& sfmData,
                                         EFeatureConstraint featureConstraint,
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

      Vec2 residual = intrinsic->residual(pose, iterTracks->second.X.homogeneous(), itObs->second.x);
      if(featureConstraint == EFeatureConstraint::SCALE && itObs->second.scale > 0.0)
      {
          // Apply the scale of the feature to get a residual value
          // relative to the feature precision.
          residual /= itObs->second.scale;
      }

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
  // note that smallest accepted angle => largest accepted cos(angle)
  const double dMaxAcceptedCosAngle = std::cos(degreeToRadian(dMinAcceptedAngle));

  using LandmarksKeysVec = std::vector<sfmData::Landmarks::key_type>;
  LandmarksKeysVec v_keys; v_keys.reserve(sfmData.structure.size());
  std::transform(sfmData.structure.cbegin(), sfmData.structure.cend(), std::back_inserter(v_keys), stl::RetrieveKey());

  LandmarksKeysVec toErase;

  #pragma omp parallel for
  for (int landmarkIndex = 0; landmarkIndex < v_keys.size(); ++landmarkIndex)
  {
    const sfmData::Observations &observations = sfmData.structure.at(v_keys[landmarkIndex]).observations;

    // create matrix for observation directions from camera to point
    Mat3X viewDirections(3, observations.size());
    Mat3X::Index i;
    sfmData::Observations::const_iterator itObs;
    
    // Greedy algorithm almost always finds an acceptable angle in 1-5 iterations (if it exists).
    // It works by greedily chasing the first larger view angle found from the current greedy index.
    // View angles have a spatial distribution, so greedily jumping over larger and larger angles
    // forces the greedy index towards the outside of the distribution.
    double dGreedyCos = 1.1;
    Mat3X::Index greedyI = 0;


    // fill matrix, optimistically checking each new entry against col(greedyI)
    for(itObs = observations.begin(), i = 0; itObs != observations.end(); ++itObs, ++i)
    {
      const sfmData::View * view = sfmData.views.at(itObs->first).get();
      const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
      const camera::IntrinsicBase * intrinsic = sfmData.intrinsics.at(view->getIntrinsicId()).get();

      viewDirections.col(i) = applyIntrinsicExtrinsic(pose, intrinsic, itObs->second.x);

      double dCosAngle = viewDirections.col(i).transpose() * viewDirections.col(greedyI);
      if (dCosAngle < dMaxAcceptedCosAngle)
      {
        break;
      }
      else if (dCosAngle < dGreedyCos)
      {
        dGreedyCos = dCosAngle;
        greedyI = i;
      }
    }

    // early exit, acceptable angle found
    if (itObs != observations.end())
    {
      continue;
    }

    // Switch to O(n^2) exhaustive search.
    // Although this is an O(n^2) loop, in practice it will almost always break very early.
    //
    // - Default value of dMinAcceptedAngle is 2 degrees. Any larger angle breaks.
    // - For landmarks with small number of views, n^2 is negligible.
    // - For landmarks with large number of views, backwards iteration means
    //     all view directions as considered as early as possible,
    //     making it difficult for a small angle to hide between views.
    //
    for(i = viewDirections.cols() - 1; i > 0; i -= 1)
    {
      // Compute and find minimum cosAngle between viewDirections[i] and all viewDirections[0:i].
      // Single statement can allow Eigen optimizations
      const double dMinCosAngle = (viewDirections.col(i).transpose() * viewDirections.leftCols(i)).minCoeff();
      if (dMinCosAngle < dMaxAcceptedCosAngle) {
        break;
      }
    }

    // acceptable angle not found
    if (i == 0)
    {
      #pragma omp critical
      toErase.push_back(v_keys[landmarkIndex]);
    }
  }

  for (IndexT key : toErase)
  {
    sfmData.structure.erase(key);
  }

  return toErase.size();
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
