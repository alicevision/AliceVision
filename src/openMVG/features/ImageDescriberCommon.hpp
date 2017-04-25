#pragma once

#include <string>
#include <vector>

namespace openMVG {
namespace features {

enum class EImageDescriberType
{
  UNKNOWN = 0,
  SIFT = 1,
  SIFT_FLOAT = 2,
#ifdef HAVE_CCTAG
  CCTAG3 = 3,
  CCTAG4 = 4,
  SIFT_CCTAG3 = 5,
  SIFT_CCTAG4 = 6,
#endif
  AKAZE_FLOAT = 7,
  AKAZE_LIOP = 8,
  AKAZE_MLDB = 9
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
