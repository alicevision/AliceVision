// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "uid.hpp"

#include <aliceVision/sfmData/View.hpp>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmData {

std::size_t computeViewUID(const View& view)
{
  std::size_t uid = 0;

  const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
  const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

  if(view.hasMetadata("Exif:ImageUniqueID") ||
     !bodySerialNumber.empty() ||
     !lensSerialNumber.empty())
  {
    stl::hash_combine(uid, view.getMetadataOrEmpty("Exif:ImageUniqueID"));
    stl::hash_combine(uid, bodySerialNumber);
    stl::hash_combine(uid, lensSerialNumber);
  }
  else
  {
    // no metadata to identify the image, fallback to the filename
    const fs::path imagePath = view.getImagePath();
    stl::hash_combine(uid, imagePath.stem().string() + imagePath.extension().string());
  }

  if(view.hasMetadata("Exif:DateTimeOriginal"))
  {
    stl::hash_combine(uid, view.getMetadataOrEmpty("Exif:DateTimeOriginal"));
    stl::hash_combine(uid, view.getMetadataOrEmpty("Exif:SubsecTimeOriginal"));
  }
  else if(view.hasMetadata("imageCounter"))
  {
    // if the view is from a video camera
    stl::hash_combine(uid, view.getMetadataOrEmpty("imageCounter"));
  }
  else
  {
    // if no original date/time, fallback to the file date/time
    stl::hash_combine(uid, view.getMetadataOrEmpty("DateTime"));
  }

  // can't use view.getWidth() and view.getHeight() directly
  // because ground truth tests and previous version datasets
  // view UID use EXIF width and height (or 0)

  if(view.hasMetadata("Exif:PixelXDimension"))
    stl::hash_combine(uid, std::stoi(view.getMetadata("Exif:PixelXDimension")));

  if(view.hasMetadata("Exif:PixelYDimension"))
    stl::hash_combine(uid, std::stoi(view.getMetadata("Exif:PixelYDimension")));

  // limit to integer to maximize compatibility (like Alembic in Maya)
  uid = std::abs((int) uid);

  return uid;
}

void updateStructureWithNewUID(Landmarks &landmarks, const std::map<std::size_t, std::size_t> &oldIdToNew)
{
  // update the id in the visibility of each 3D point
  for(auto &iter : landmarks)
  {
    Landmark& currentLandmark = iter.second;
    
    // the new observations where to copy the existing ones
    // (needed as the key of the map is the idview)
    Observations newObservations;
    
    for(const auto &iterObs : currentLandmark.observations)
    {
      const auto idview = iterObs.first;
      const Observation &obs = iterObs.second;

      newObservations.emplace(oldIdToNew.at(idview), obs);
    }
    
    assert(currentLandmark.observations.size() == newObservations.size());
    currentLandmark.observations.swap(newObservations);
  }  
}


void sanityCheckLandmarks(const Landmarks &landmarks, const Views &views)
{
  for(const auto &iter : landmarks)
  {
    const Landmark& currentLandmark = iter.second;
    for(const auto &iterObs : currentLandmark.observations)
    {
      const auto idview = iterObs.first;

      // there must be a view with that id (in the map) and the view must have 
      // the same id (the member)
      assert(views.count(idview) == 1);
      assert(views.at(idview)->getViewId() == idview);
    }
  }  
}

void regenerateUID(SfMData &sfmdata, std::map<std::size_t, std::size_t> &oldIdToNew, bool sanityCheck)
{
  // if the views are empty, nothing to be done. 
  if(sfmdata.getViews().empty())
    return;
  
  regenerateViewUIDs(sfmdata.views, oldIdToNew);
  
  if(!sanityCheck)
    return;
  
  sanityCheckLandmarks(sfmdata.getLandmarks(), sfmdata.getViews());
  
  sanityCheckLandmarks(sfmdata.getControlPoints(), sfmdata.getViews());
  
}


void regenerateViewUIDs(Views &views, std::map<std::size_t, std::size_t> &oldIdToNew)
{
  // if the views are empty, nothing to be done. 
  if(views.empty())
    return;
  
  Views newViews;

  for(auto const &iter : views)
  {
    const View& currentView = *iter.second.get();

    // compute the view UID
    const std::size_t uid = computeViewUID(currentView);

    // update the mapping
    assert(oldIdToNew.count(currentView.getViewId()) == 0);
    oldIdToNew.emplace(currentView.getViewId(), uid);
    
    // add the view to the new map using the uid as key and change the id
    assert(newViews.count(uid) == 0);
    newViews.emplace(uid, iter.second);
    newViews[uid]->setViewId(uid);
  }
  
  assert(newViews.size() == views.size());
  views.swap(newViews);
}

}
}
