// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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

EDESCRIBER_PRESET describerPreset_stringToEnum(const std::string& sPreset)
{
  if(sPreset == "LOW")
    return LOW_PRESET;
  if (sPreset == "MEDIUM")
    return MEDIUM_PRESET;
  if(sPreset == "NORMAL")
    return NORMAL_PRESET;
  if (sPreset == "HIGH")
    return HIGH_PRESET;
  if (sPreset == "ULTRA")
    return ULTRA_PRESET;
  throw std::invalid_argument("Invalid descriptor preset: " + sPreset);
}

std::string describerPreset_enumToString(const EDESCRIBER_PRESET preset)
{
  if(preset == LOW_PRESET)
    return "LOW";
  if (preset == MEDIUM_PRESET)
    return "MEDIUM";
  if(preset == NORMAL_PRESET)
    return "NORMAL";
  if (preset == HIGH_PRESET)
    return "HIGH";
  if (preset == ULTRA_PRESET)
    return "ULTRA";
  throw std::invalid_argument("Unrecognized EDESCRIBER_PRESET "+std::to_string(preset));
}

std::ostream& operator<<(std::ostream& os, EDESCRIBER_PRESET p)
{
    return os << describerPreset_enumToString(p);
}

std::istream& operator>>(std::istream& in, EDESCRIBER_PRESET& p)
{
    std::string token;
    in >> token;
    p = describerPreset_stringToEnum(token);
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
