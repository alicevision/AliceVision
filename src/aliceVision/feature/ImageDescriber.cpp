// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageDescriber.hpp"

#include <aliceVision/config.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT_vlfeatFloat.hpp>
#include <aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#include <aliceVision/feature/cctag/ImageDescriber_CCTAG.hpp>
#endif //ALICEVISION_HAVE_CCTAG

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
#include <aliceVision/feature/openCV/ImageDescriber_SIFT_OCV.hpp>
#endif //ALICEVISION_HAVE_OCVSIFT
#include <aliceVision/feature/openCV/ImageDescriber_AKAZE_OCV.hpp>
#endif //ALICEVISION_HAVE_OPENCV

#include <boost/filesystem.hpp>

#include <algorithm>
#include <stdexcept>

namespace fs = boost::filesystem;

namespace aliceVision{
namespace feature{

EImageDescriberPreset EImageDescriberPreset_stringToEnum(const std::string& imageDescriberPreset)
{
  std::string preset = imageDescriberPreset;
  std::transform(preset.begin(), preset.end(), preset.begin(), ::tolower); //tolower

  if(preset == "low")    return EImageDescriberPreset::LOW;
  if(preset == "medium") return EImageDescriberPreset::MEDIUM;
  if(preset == "normal") return EImageDescriberPreset::NORMAL;
  if(preset == "high")   return EImageDescriberPreset::HIGH;
  if(preset == "ultra")  return EImageDescriberPreset::ULTRA;

  throw std::invalid_argument("Invalid descriptor preset: " + preset);
}

std::string EImageDescriberPreset_enumToString(const EImageDescriberPreset imageDescriberPreset)
{
  if(imageDescriberPreset == EImageDescriberPreset::LOW)    return "low";
  if(imageDescriberPreset == EImageDescriberPreset::MEDIUM) return "medium";
  if(imageDescriberPreset == EImageDescriberPreset::NORMAL) return "normal";
  if(imageDescriberPreset == EImageDescriberPreset::HIGH)   return "high";
  if(imageDescriberPreset == EImageDescriberPreset::ULTRA)  return "ultra";

  throw std::invalid_argument("Unrecognized EImageDescriberPreset");
}

std::ostream& operator<<(std::ostream& os, EImageDescriberPreset p)
{
  return os << EImageDescriberPreset_enumToString(p);
}

std::istream& operator>>(std::istream& in, EImageDescriberPreset& p)
{
  std::string token;
  in >> token;
  p = EImageDescriberPreset_stringToEnum(token);
  return in;
}

void ImageDescriber::Save(const Regions* regions, const std::string& sfileNameFeats, const std::string& sfileNameDescs) const
{
  const fs::path bFeatsPath = fs::path(sfileNameFeats);
  const fs::path bDescsPath = fs::path(sfileNameDescs);
  const std::string tmpFeatsPath = (bFeatsPath.parent_path() / bFeatsPath.stem()).string() + "." + fs::unique_path().string() + bFeatsPath.extension().string();
  const std::string tmpDescsPath = (bDescsPath.parent_path() / bDescsPath.stem()).string() + "." + fs::unique_path().string() + bDescsPath.extension().string();

  regions->Save(tmpFeatsPath, tmpDescsPath);

  // rename temporay filenames
  fs::rename(tmpFeatsPath, sfileNameFeats);
  fs::rename(tmpDescsPath, sfileNameDescs);
}

std::unique_ptr<ImageDescriber> createImageDescriber(EImageDescriberType imageDescriberType)
{
  std::unique_ptr<ImageDescriber> describerPtr;
  
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:           describerPtr.reset(new ImageDescriber_SIFT(SiftParams(), true)); break;
    case EImageDescriberType::SIFT_FLOAT:     describerPtr.reset(new ImageDescriber_SIFT_vlfeatFloat(SiftParams())); break;
    case EImageDescriberType::SIFT_UPRIGHT:   describerPtr.reset(new ImageDescriber_SIFT(SiftParams(), false)); break;
    case EImageDescriberType::AKAZE:          describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEOptions(), feature::AKAZE_MSURF))); break;
    case EImageDescriberType::AKAZE_MLDB:     describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEOptions(), feature::AKAZE_MLDB))); break;
    case EImageDescriberType::AKAZE_LIOP:     describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEOptions(), feature::AKAZE_LIOP))); break;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:      describerPtr.reset(new ImageDescriber_CCTAG(3)); break;
    case EImageDescriberType::CCTAG4:      describerPtr.reset(new ImageDescriber_CCTAG(4)); break;
#endif //ALICEVISION_HAVE_CCTAG

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
  case EImageDescriberType::SIFT_OCV:      describerPtr.reset(new ImageDescriber_SIFT_openCV()); break;
#endif //ALICEVISION_HAVE_OCVSIFT
  case EImageDescriberType::AKAZE_OCV:     describerPtr.reset(new ImageDescriber_AKAZE_OCV()); break;
#endif //ALICEVISION_HAVE_OPENCV
    
    default: throw std::out_of_range("Invalid imageDescriber enum");
  }
  assert(describerPtr != nullptr);

  return describerPtr;
}

}//namespace feature
}//namespace aliceVision
