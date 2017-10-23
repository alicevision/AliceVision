// This file is part of the AliceVision project.
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

#include <stdexcept>

namespace aliceVision{
namespace feature{

EImageDescriberPreset EImageDescriberPreset_stringToEnum(const std::string& sPreset)
{
  if(sPreset == "LOW")
    return EImageDescriberPreset::LOW;
  if (sPreset == "MEDIUM")
    return EImageDescriberPreset::MEDIUM;
  if(sPreset == "NORMAL")
    return EImageDescriberPreset::NORMAL;
  if (sPreset == "HIGH")
    return EImageDescriberPreset::HIGH;
  if (sPreset == "ULTRA")
    return EImageDescriberPreset::ULTRA;
  throw std::invalid_argument("Invalid descriptor preset: " + sPreset);
}

std::string EImageDescriberPreset_enumToString(const EImageDescriberPreset preset)
{
  if(preset == EImageDescriberPreset::LOW)
    return "LOW";
  if (preset == EImageDescriberPreset::MEDIUM)
    return "MEDIUM";
  if(preset == EImageDescriberPreset::NORMAL)
    return "NORMAL";
  if (preset == EImageDescriberPreset::HIGH)
    return "HIGH";
  if (preset == EImageDescriberPreset::ULTRA)
    return "ULTRA";
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

std::unique_ptr<ImageDescriber> createImageDescriber(EImageDescriberType imageDescriberType)
{
  std::unique_ptr<ImageDescriber> describerPtr;
  
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:        describerPtr.reset(new ImageDescriber_SIFT(SiftParams())); break;
    case EImageDescriberType::SIFT_FLOAT:  describerPtr.reset(new ImageDescriber_SIFT_vlfeatFloat(SiftParams())); break;
    case EImageDescriberType::AKAZE:       describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEConfig(), feature::AKAZE_MSURF))); break;
    case EImageDescriberType::AKAZE_MLDB:  describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEConfig(), feature::AKAZE_MLDB))); break;
    case EImageDescriberType::AKAZE_LIOP:  describerPtr.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEConfig(), feature::AKAZE_LIOP))); break;

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
