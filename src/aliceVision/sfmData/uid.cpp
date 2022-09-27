// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "uid.hpp"

#include <aliceVision/sfmData/View.hpp>

#include <boost/algorithm/string/case_conv.hpp> 
#include <boost/filesystem.hpp>


namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmData {

std::size_t computeViewUID(const View& view)
{
  std::size_t uid = 0;
  const fs::path imagePath = view.getImagePath();

  {
      std::string ext = imagePath.extension().string();
      boost::to_lower(ext);
      stl::hash_combine(uid, ext);
  }

  const std::string& bodySerialNumber = view.getMetadataBodySerialNumber();
  const std::string& lensSerialNumber = view.getMetadataLensSerialNumber();

  const bool hasImageUniqueID = view.hasMetadata({"Exif:ImageUniqueID", "ImageUniqueID"});
  if(hasImageUniqueID)
  {
    stl::hash_combine(uid, view.getMetadata({"Exif:ImageUniqueID", "ImageUniqueID"}));
  }
  else
  {
    // No uid in the metadata to identify the image, fallback to the filename.
    // File extension is already used in the uid, so add the stem.
    std::string stem = imagePath.stem().string();
    boost::to_lower(stem);
    stl::hash_combine(uid, stem);
  }

  if(!bodySerialNumber.empty() || !lensSerialNumber.empty())
  {
    stl::hash_combine(uid, bodySerialNumber);
    stl::hash_combine(uid, lensSerialNumber);
  }
  else if(!hasImageUniqueID)
  {
    // No metadata to identify the device, fallback to the folder path of the file.
    // The image will NOT be relocatable in this particular case.
    stl::hash_combine(uid, imagePath.parent_path().generic_string());
  }

  bool hasTimeOrCounterMetadata = false;
  if(view.hasMetadataDateTimeOriginal())
  {
    stl::hash_combine(uid, view.getMetadataDateTimeOriginal());
    stl::hash_combine(uid, view.getMetadata({"Exif:SubsecTimeOriginal", "SubsecTimeOriginal"}));
    hasTimeOrCounterMetadata = true;
  }

  if(view.hasMetadata({"imageCounter"}))
  {
    // if the view is from a video camera
    stl::hash_combine(uid, view.getMetadata({"imageCounter"}));
    hasTimeOrCounterMetadata = true;
  }

  if(hasTimeOrCounterMetadata)
  {
    // already added to the uid
  }
  else if(view.hasMetadata({"DateTime"}))
  {
    // if no original date/time, fallback to the file date/time
    stl::hash_combine(uid, view.getMetadata({"DateTime"}));
  }
  else
  {
    // if no original date/time, fallback to the file date/time
    std::time_t t = fs::last_write_time(imagePath);
    stl::hash_combine(uid, t);
  }

  // cannot use view.getWidth() and view.getHeight() directly
  // because ground truth tests and previous version datasets
  // view UID use EXIF width and height (or 0)

  if(view.hasMetadata({"Exif:PixelXDimension", "PixelXDimension"}))
    stl::hash_combine(uid, std::stoi(view.getMetadata({"Exif:PixelXDimension", "PixelXDimension"})));

  if(view.hasMetadata({"Exif:PixelYDimension", "PixelYDimension"}))
    stl::hash_combine(uid, std::stoi(view.getMetadata({"Exif:PixelYDimension", "PixelYDimension"})));

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
