#include "ImageDescriberCommon.hpp"

#include "openMVG/types.hpp"
#include "openMVG/stl/split.hpp"

#include <exception>
#include <cassert>


namespace openMVG {
namespace features {

std::string EImageDescriberType_enumToString(EImageDescriberType imageDescriberType)
{
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:          return "SIFT";
    case EImageDescriberType::SIFT_FLOAT:    return "SIFT_FLOAT";
    case EImageDescriberType::AKAZE:         return "AKAZE";
    case EImageDescriberType::AKAZE_LIOP:    return "AKAZE_LIOP";
    case EImageDescriberType::AKAZE_MLDB:    return "AKAZE_MLDB";
    
#ifdef HAVE_CCTAG
    case EImageDescriberType::CCTAG3:        return "CCTAG3";
    case EImageDescriberType::CCTAG4:        return "CCTAG4";
#endif //HAVE_CCTAG

#ifdef HAVE_OPENCV
#ifdef USE_OCVSIFT
    case EImageDescriberType::SIFT_OCV:      return "SIFT_OCV";
#endif
    case EImageDescriberType::AKAZE_OCV:     return "AKAZE_OCV";
#endif//HAVE_OPENCV

    case EImageDescriberType::UNKNOWN:       return "UNKNOWN";
    case EImageDescriberType::UNINITIALIZED: break; // Should throw an error.
  }
  throw std::out_of_range("Invalid imageDescriber enum");
}

EImageDescriberType EImageDescriberType_stringToEnum(const std::string& imageDescriberType)
{
  if(imageDescriberType == "SIFT")        return EImageDescriberType::SIFT;
  if(imageDescriberType == "SIFT_FLOAT")  return EImageDescriberType::SIFT_FLOAT;
  if(imageDescriberType == "AKAZE")       return EImageDescriberType::AKAZE;
  if(imageDescriberType == "AKAZE_LIOP")  return EImageDescriberType::AKAZE_LIOP;
  if(imageDescriberType == "AKAZE_MLDB")  return EImageDescriberType::AKAZE_MLDB;
  
#ifdef HAVE_CCTAG
  if(imageDescriberType == "CCTAG3")      return EImageDescriberType::CCTAG3;
  if(imageDescriberType == "CCTAG4")      return EImageDescriberType::CCTAG4;
#endif //HAVE_CCTAG

#ifdef HAVE_OPENCV
#ifdef USE_OCVSIFT
  if(imageDescriberType == "SIFT_OCV")    return EImageDescriberType::SIFT_OCV;
#endif
  if(imageDescriberType == "AKAZE_OCV")   return EImageDescriberType::AKAZE_OCV;
#endif//HAVE_OPENCV

  if(imageDescriberType == "UNKNOWN")     return EImageDescriberType::UNKNOWN;
  // UNINITIALIZED should throw an error.
  throw std::out_of_range("Invalid imageDescriber : " + imageDescriberType);
}

std::vector<EImageDescriberType> EImageDescriberType_stringToEnums(const std::string& describerMethods)
{
  std::vector<EImageDescriberType> out;
  std::vector<std::string> describerMethodsVec;
  stl::split(describerMethods, ",", describerMethodsVec);

  for(const auto& describerMethod: describerMethodsVec)
  {
    out.push_back(EImageDescriberType_stringToEnum(describerMethod));
  }
  return out;
}

} // namespace features
} // namespace openMVG
