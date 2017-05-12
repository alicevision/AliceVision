#pragma once

#include <openMVG/config.hpp>
#include <string>
#include <vector>

namespace openMVG {
namespace features {

enum class EImageDescriberType: unsigned char
{
  UNKNOWN = 0
  , UNINITIALIZED = 1
  , SIFT = 10
  , SIFT_FLOAT = 11

  , AKAZE = 20
  , AKAZE_LIOP = 21
  , AKAZE_MLDB = 22

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
  , CCTAG3 = 30
  , CCTAG4 = 31
#endif

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#if OPENMVG_IS_DEFINED(OPENMVG_USE_OCVSIFT)
  , SIFT_OCV = 40
#endif
  , AKAZE_OCV = 41
#endif
};

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
  * @return
  */
std::vector<EImageDescriberType> EImageDescriberType_stringToEnums(const std::string& describerMethods);

  
} // namespace features
} // namespace openMVG
