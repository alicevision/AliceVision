#pragma once

#include <openMVG/config.hpp>
#include <string>
#include <vector>
#include <stdexcept>

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
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
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
  * @return EImageDescriberType vector
  */
std::vector<EImageDescriberType> EImageDescriberType_stringToEnums(const std::string& describerMethods);

/**
 * @brief getStrongSupportCoeff
 * @param imageDescriberType
 * @return strong support coeff score
 */
inline float getStrongSupportCoeff(EImageDescriberType imageDescriberType)
{
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:          return 0.14f;
    case EImageDescriberType::SIFT_FLOAT:    return 0.14f;
    case EImageDescriberType::AKAZE:         return 0.14f;
    case EImageDescriberType::AKAZE_LIOP:    return 0.14f;
    case EImageDescriberType::AKAZE_MLDB:    return 0.14f;

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:        return 1.0f;
    case EImageDescriberType::CCTAG4:        return 1.0f;
#endif

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
    case EImageDescriberType::SIFT_OCV:      return 0.14f;
#endif //OPENMVG_HAVE_OCVSIFT
    case EImageDescriberType::AKAZE_OCV:     return 0.14f;
#endif //OPENMVG_HAVE_OPENCV

    case EImageDescriberType::UNKNOWN:       return 1.0f;
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

} // namespace features
} // namespace openMVG
