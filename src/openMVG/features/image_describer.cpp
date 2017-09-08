// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "image_describer.hpp"

#include <aliceVision/config.hpp>
#include <aliceVision/features/image_describer_akaze.hpp>
#include <aliceVision/features/sift/SIFT_describer.hpp>
#include <aliceVision/features/sift/SIFT_vlfeat_float_describer.hpp>
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
#include <aliceVision/features/cctag/CCTAG_describer.hpp>
#endif //OPENMVG_HAVE_CCTAG
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#include <aliceVision/features/openCV/AKAZE_openCV_describer.hpp>
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
#include <aliceVision/features/openCV/SIFT_openCV_describer.hpp>
#endif //OPENMVG_HAVE_OCVSIFT
#endif //OPENMVG_HAVE_OPENCV

#include <stdexcept>

namespace aliceVision{
namespace features{

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

std::unique_ptr<Image_describer> createImageDescriber(EImageDescriberType imageDescriberType)
{
  std::unique_ptr<Image_describer> describerPtr;
  
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:        describerPtr.reset(new SIFT_ImageDescriber(SiftParams())); break;
    case EImageDescriberType::SIFT_FLOAT:  describerPtr.reset(new SIFT_vlfeat_float_ImageDescriber(SiftParams())); break;
    case EImageDescriberType::AKAZE:       describerPtr.reset(new AKAZE_Image_describer(AKAZEParams(AKAZEConfig(), features::AKAZE_MSURF))); break;
    case EImageDescriberType::AKAZE_MLDB:  describerPtr.reset(new AKAZE_Image_describer(AKAZEParams(AKAZEConfig(), features::AKAZE_MLDB))); break;
    case EImageDescriberType::AKAZE_LIOP:  describerPtr.reset(new AKAZE_Image_describer(AKAZEParams(AKAZEConfig(), features::AKAZE_LIOP))); break;

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:      describerPtr.reset(new CCTAG_Image_describer(3)); break;
    case EImageDescriberType::CCTAG4:      describerPtr.reset(new CCTAG_Image_describer(4)); break;
#endif //OPENMVG_HAVE_CCTAG

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
  case EImageDescriberType::SIFT_OCV:      describerPtr.reset(new SIFT_openCV_ImageDescriber()); break;
#endif //OPENMVG_HAVE_OCVSIFT
  case EImageDescriberType::AKAZE_OCV:     describerPtr.reset(new AKAZE_openCV_ImageDescriber()); break;
#endif //OPENMVG_HAVE_OPENCV
    
    default: throw std::out_of_range("Invalid imageDescriber enum");
  }
  assert(describerPtr != nullptr);

  return describerPtr;
}

}//namespace features
}//namespace aliceVision
