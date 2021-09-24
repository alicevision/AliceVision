// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>

#include <string>
#include <vector>
#include <stdexcept>

namespace aliceVision {
namespace feature {

enum class EImageDescriberType: unsigned char
{
  UNKNOWN = 0
  , UNINITIALIZED = 1
  , SIFT = 10
  , SIFT_FLOAT = 11
  , SIFT_UPRIGHT = 12
  , DSPSIFT = 13

  , AKAZE = 20
  , AKAZE_LIOP = 21
  , AKAZE_MLDB = 22

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
  , CCTAG3 = 30
  , CCTAG4 = 31
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
  , SIFT_OCV = 40
#endif
  , AKAZE_OCV = 41
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
  , APRILTAG16H5 = 50
#endif
};

/**
 * @brief get informations about each describer type
 * @return String
 */
std::string EImageDescriberType_informations();

/**
 * @brief convert an enum EImageDescriberType to its corresponding string
 * @param EImageDescriberType 
 * @return String
 */
std::string EImageDescriberType_enumToString(EImageDescriberType imageDescriberType);

/**
 * @brief convert a string imageDescriber to its corresponding enum EImageDescriberType
 * @param String
 * @return EImageDescriberType
 */
EImageDescriberType EImageDescriberType_stringToEnum(const std::string& imageDescriberType);

 /**
  * @brief EImageDescriberType_stringToEnums
  * @param describerMethods
  * @return EImageDescriberType vector
  */
std::vector<EImageDescriberType> EImageDescriberType_stringToEnums(const std::string& describerMethods);

inline bool isMarker(EImageDescriberType imageDescriberType)
{
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    if (imageDescriberType == EImageDescriberType::CCTAG3 || imageDescriberType == EImageDescriberType::CCTAG4) {
      return true;
    }
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    if (imageDescriberType == EImageDescriberType::APRILTAG16H5) {
      return true;
    }
#endif
  return false;
}

/**
 * @brief getStrongSupportCoeff
 * @param imageDescriberType
 * @return strong support coeff score
 */
inline float getStrongSupportCoeff(EImageDescriberType imageDescriberType)
{
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:
    case EImageDescriberType::SIFT_FLOAT:
    case EImageDescriberType::SIFT_UPRIGHT:
    case EImageDescriberType::DSPSIFT:
    case EImageDescriberType::AKAZE:
    case EImageDescriberType::AKAZE_LIOP:
    case EImageDescriberType::AKAZE_MLDB:
        return 0.14f;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:
    case EImageDescriberType::CCTAG4:
        return 1.0f;
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    case EImageDescriberType::APRILTAG16H5:
        return 1.0f;
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
    case EImageDescriberType::SIFT_OCV:      return 0.14f;
#endif //ALICEVISION_HAVE_OCVSIFT
    case EImageDescriberType::AKAZE_OCV:     return 0.14f;
#endif //ALICEVISION_HAVE_OPENCV

    case EImageDescriberType::UNKNOWN:
        return 1.0f;
    case EImageDescriberType::UNINITIALIZED: break; // Should throw an error.
  }
  throw std::out_of_range("Invalid imageDescriber enum");
}

inline std::ostream& operator<<(std::ostream& os, const EImageDescriberType imageDescriberType)
{
  os << EImageDescriberType_enumToString(imageDescriberType);
  return os;
}

inline std::istream& operator>>(std::istream& in, EImageDescriberType &imageDescriberType)
{
  std::string token;
  in >> token;
  imageDescriberType = EImageDescriberType_stringToEnum(token);
  return in;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<EImageDescriberType> &imageDescriberTypes)
{
  for(const EImageDescriberType descType : imageDescriberTypes)
    os << descType;
  return os;
}

inline std::istream& operator>>(std::istream &in, std::vector<EImageDescriberType> &imageDescriberTypes)
{
  std::string token;
  in >> token;
  imageDescriberTypes = EImageDescriberType_stringToEnums(token);
  return in;
}

} // namespace feature
} // namespace aliceVision
