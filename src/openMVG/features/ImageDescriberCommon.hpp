#pragma once

#include <string>
#include <vector>

namespace openMVG {
namespace features {

enum class EImageDescriberType
{
  UNKNOWN = 0,
  UNINITIALIZED = 1,
  SIFT = 2,
  SIFT_FLOAT = 3,
#ifdef HAVE_CCTAG
  CCTAG3 = 4,
  CCTAG4 = 5,
  SIFT_CCTAG3 = 6,
  SIFT_CCTAG4 = 7,
#endif
  AKAZE = 8,
  AKAZE_LIOP = 9,
  AKAZE_MLDB = 10
};
  
/**
 * @brief convert an enum EImageDescriberType to it's corresponding string
 * @param EImageDescriberType 
 * @return String
 */
std::string EImageDescriberType_enumToString(EImageDescriberType imageDescriberType);


/**
 * @brief convert a string imageDescriber to it's corresponding enum EImageDescriberType
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
