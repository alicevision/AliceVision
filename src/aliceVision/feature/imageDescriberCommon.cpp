// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "imageDescriberCommon.hpp"
#include <aliceVision/types.hpp>

#include <boost/algorithm/string.hpp>

#include <cassert>


namespace aliceVision {
namespace feature {

std::string EImageDescriberType_informations()
{
  return  "Describer types used to describe an image:\n"
          "* sift: Scale-invariant feature transform.\n"
          "* sift_float: SIFT stored as float.\n"
          "* sift_upright: SIFT with upright feature.\n"
          "* akaze: A-KAZE with floating point descriptors.\n"
          "* akaze_liop: A-KAZE with Local Intensity Order Pattern descriptors.\n"
          "* akaze_mldb: A-KAZE with Modified-Local Difference Binary descriptors.\n"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
          "* cctag3: Concentric circles markers with 3 crowns.\n"
          "* cctag4: Concentric circles markers with 4 crowns.\n"
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
          "* tag16h5: AprilTag family tag16h5.\n"
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
          "* sift_ocv: OpenCV implementation of SIFT describer.\n"
#endif
          "* akaze_ocv: OpenCV implementation of A-KAZE describer.\n"
#endif
          "";
}

std::string EImageDescriberType_enumToString(EImageDescriberType imageDescriberType)
{
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:          return "sift";
    case EImageDescriberType::SIFT_FLOAT:    return "sift_float";
    case EImageDescriberType::SIFT_UPRIGHT:  return "sift_upright";

    case EImageDescriberType::DSPSIFT:       return "dspsift";

    case EImageDescriberType::AKAZE:         return "akaze";
    case EImageDescriberType::AKAZE_LIOP:    return "akaze_liop";
    case EImageDescriberType::AKAZE_MLDB:    return "akaze_mldb";
    
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:        return "cctag3";
    case EImageDescriberType::CCTAG4:        return "cctag4";
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    case EImageDescriberType::APRILTAG16H5:  return "tag16h5";
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
    case EImageDescriberType::SIFT_OCV:      return "sift_ocv";
#endif //ALICEVISION_HAVE_OCVSIFT
    case EImageDescriberType::AKAZE_OCV:     return "akaze_ocv";
#endif //ALICEVISION_HAVE_OPENCV

    case EImageDescriberType::UNKNOWN:       return "unknown";
    case EImageDescriberType::UNINITIALIZED: break; // Should throw an error.
  }
  throw std::out_of_range("Invalid imageDescriber enum: " + std::to_string(int(imageDescriberType)));
}

EImageDescriberType EImageDescriberType_stringToEnum(const std::string& imageDescriberType)
{
  std::string type = imageDescriberType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); //tolower

  if(type == "sift")          return EImageDescriberType::SIFT;
  if(type == "sift_float")    return EImageDescriberType::SIFT_FLOAT;
  if(type == "sift_upright")  return EImageDescriberType::SIFT_UPRIGHT;

  if(type == "dspsift")       return EImageDescriberType::DSPSIFT;

  if(type == "akaze")         return EImageDescriberType::AKAZE;
  if(type == "akaze_liop")    return EImageDescriberType::AKAZE_LIOP;
  if(type == "akaze_mldb")    return EImageDescriberType::AKAZE_MLDB;
  
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  if(type == "cctag3")        return EImageDescriberType::CCTAG3;
  if(type == "cctag4")        return EImageDescriberType::CCTAG4;
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
  if(type == "tag16h5")       return EImageDescriberType::APRILTAG16H5;
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
  if(type == "sift_ocv")      return EImageDescriberType::SIFT_OCV;
#endif //ALICEVISION_HAVE_OCVSIFT
  if(type == "akaze_ocv")     return EImageDescriberType::AKAZE_OCV;
#endif //ALICEVISION_HAVE_OPENCV

  if(type == "unknown")       return EImageDescriberType::UNKNOWN;
  // UNINITIALIZED should throw an error.
  throw std::out_of_range("Invalid imageDescriber : " + imageDescriberType);
}

std::vector<EImageDescriberType> EImageDescriberType_stringToEnums(const std::string& describerMethods)
{
  std::vector<EImageDescriberType> out;
  std::vector<std::string> describerMethodsVec;
  boost::split(describerMethodsVec, describerMethods, boost::is_any_of(","));

  for(const auto& describerMethod: describerMethodsVec)
  {
    out.push_back(EImageDescriberType_stringToEnum(describerMethod));
  }
  return out;
}

} // namespace feature
} // namespace aliceVision
