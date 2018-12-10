// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMData.hpp"

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace sfmData {

using namespace aliceVision::geometry;
using namespace aliceVision::camera;
using namespace aliceVision::image;

namespace fs = boost::filesystem;

bool SfMData::operator==(const SfMData& other) const {

  // Views
  if(views.size() != other.views.size())
    return false;

  for(Views::const_iterator it = views.begin(); it != views.end(); ++it)
  {
      const View& view1 = *(it->second.get());
      const View& view2 = *(other.views.at(it->first).get());
      if(!(view1 == view2))
        return false;

      // Image paths
      if(view1.getImagePath() != view2.getImagePath())
        return false;
  }

  // Poses
  if((_poses != other._poses))
    return false;

  // Rigs
  if(_rigs != other._rigs)
    return false;

  // Intrinsics
  if(intrinsics.size() != other.intrinsics.size())
    return false;

  Intrinsics::const_iterator it = intrinsics.begin();
  Intrinsics::const_iterator otherIt = other.intrinsics.begin();
  for(; it != intrinsics.end() && otherIt != other.intrinsics.end(); ++it, ++otherIt)
  {
      // Index
      if(it->first != otherIt->first)
        return false;

      // Intrinsic
      camera::IntrinsicBase& intrinsic1 = *(it->second.get());
      camera::IntrinsicBase& intrinsic2 = *(otherIt->second.get());
      if(!(intrinsic1 == intrinsic2))
        return false;
  }

  // Points IDs are not preserved
  if(structure.size() != other.structure.size())
    return false;

  Landmarks::const_iterator landMarkIt = structure.begin();
  Landmarks::const_iterator otherLandmarkIt = other.structure.begin();
  for(; landMarkIt != structure.end() && otherLandmarkIt != other.structure.end(); ++landMarkIt, ++otherLandmarkIt)
  {
      // Points IDs are not preserved
      // Landmark
      const Landmark& landmark1 = landMarkIt->second;
      const Landmark& landmark2 = otherLandmarkIt->second;
      if(!(landmark1 == landmark2))
        return false;
  }

  // Control points
  if(control_points != other.control_points)
    return false;

  // Root path can be reseted during exports

  return true;

}

std::vector<std::string> SfMData::getFeaturesFolders() const
{
  fs::path sfmFolder = fs::path(_absolutePath).parent_path();
  std::vector<std::string> absolutePaths(_featuresFolders.size());
  for(int i = 0; i < absolutePaths.size(); ++i)
    absolutePaths.at(i) = fs::canonical(fs::path(_featuresFolders.at(i)), sfmFolder).string();
  return absolutePaths;
}

std::vector<std::string> SfMData::getMatchesFolders() const
{
  fs::path sfmFolder = fs::path(_absolutePath).parent_path();
  std::vector<std::string> absolutePaths(_matchesFolders.size());
  for(int i = 0; i < absolutePaths.size(); ++i)
    absolutePaths.at(i) = fs::canonical(fs::path(_matchesFolders.at(i)), sfmFolder).string();
  return absolutePaths;
}

std::set<IndexT> SfMData::getValidViews() const
{
  std::set<IndexT> valid_idx;
  for (Views::const_iterator it = views.begin();
    it != views.end(); ++it)
  {
    const View * v = it->second.get();
    if (isPoseAndIntrinsicDefined(v))
    {
      valid_idx.insert(v->getViewId());
    }
  }
  return valid_idx;
}

std::set<IndexT> SfMData::getReconstructedIntrinsics() const
{
  std::set<IndexT> valid_idx;
  for (Views::const_iterator it = views.begin();
    it != views.end(); ++it)
  {
    const View * v = it->second.get();
    if (isPoseAndIntrinsicDefined(v))
    {
      valid_idx.insert(v->getIntrinsicId());
    }
  }
  return valid_idx;
}

void SfMData::setPose(const View& view, const CameraPose& absolutePose)
{
  //const bool knownPose = existsPose(view);
  CameraPose& viewPose = _poses[view.getPoseId()];

  // pose dedicated for this view (independant from rig, even if it is potentially part of a rig)
  if(view.isPoseIndependant())
  {
    viewPose.setTransform(absolutePose.getTransform());
    return;
  }

  // initialized rig
  if(view.getRigId() != UndefinedIndexT)
  {
    const Rig& rig = _rigs.at(view.getRigId());
    RigSubPose& subPose = getRigSubPose(view);

    viewPose.setTransform(subPose.pose.inverse() * absolutePose.getTransform());
    return;
  }

  throw std::runtime_error("SfMData::setPose: dependant view pose not part of an initialized rig.");
}


void SfMData::combine(const SfMData& sfmData)
{
  if(!_rigs.empty() && !sfmData._rigs.empty())
    throw std::runtime_error("Can't combine two SfMData with rigs");

  // feature folder
  _featuresFolders.insert(_featuresFolders.end(), sfmData._featuresFolders.begin(), sfmData._featuresFolders.end());

  // matching folder
  _matchesFolders.insert(_matchesFolders.end(), sfmData._matchesFolders.begin(), sfmData._matchesFolders.end());

  // views
  views.insert(sfmData.views.begin(), sfmData.views.end());

  // intrinsics
  intrinsics.insert(sfmData.intrinsics.begin(), sfmData.intrinsics.end());

  // poses
  _poses.insert(sfmData._poses.begin(), sfmData._poses.end());

  // rigs
  _rigs.insert(sfmData._rigs.begin(), sfmData._rigs.end());

  // structure
  structure.insert(sfmData.structure.begin(), sfmData.structure.end());

  // control points
  control_points.insert(sfmData.control_points.begin(), sfmData.control_points.end());
}

} // namespace sfmData
} // namespace aliceVision
