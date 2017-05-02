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
    case EImageDescriberType::SIFT:        return "SIFT";
    case EImageDescriberType::SIFT_FLOAT:  return "SIFT_FLOAT";
    case EImageDescriberType::AKAZE_FLOAT: return "AKAZE_FLOAT";
    case EImageDescriberType::AKAZE_LIOP:  return "AKAZE_LIOP";
    case EImageDescriberType::AKAZE_MLDB:  return "AKAZE_MLDB";
    
#ifdef HAVE_CCTAG
    case EImageDescriberType::CCTAG3:      return "CCTAG3";
    case EImageDescriberType::CCTAG4:      return "CCTAG4";
    case EImageDescriberType::SIFT_CCTAG3: return "SIFT_CCTAG3";
    case EImageDescriberType::SIFT_CCTAG4: return "SIFT_CCTAG4";
#endif //HAVE_CCTAG
    case EImageDescriberType::UNKNOWN: return "UNKNOWN";
    case EImageDescriberType::UNINITIALIZED: break; // Should throw an error.
  }
  throw std::out_of_range("Invalid imageDescriber enum");
}

EImageDescriberType EImageDescriberType_stringToEnum(const std::string& imageDescriberType)
{
  if(imageDescriberType == "SIFT")        return EImageDescriberType::SIFT;
  if(imageDescriberType == "SIFT_FLOAT")  return EImageDescriberType::SIFT_FLOAT;
  if(imageDescriberType == "AKAZE_FLOAT") return EImageDescriberType::AKAZE_FLOAT;
  if(imageDescriberType == "AKAZE_LIOP")  return EImageDescriberType::AKAZE_LIOP;
  if(imageDescriberType == "AKAZE_MLDB")  return EImageDescriberType::AKAZE_MLDB;
  
#ifdef HAVE_CCTAG
  if(imageDescriberType == "CCTAG3")      return EImageDescriberType::CCTAG3;
  if(imageDescriberType == "CCTAG4")      return EImageDescriberType::CCTAG4;
  if(imageDescriberType == "SIFT_CCTAG3") return EImageDescriberType::SIFT_CCTAG3;
  if(imageDescriberType == "SIFT_CCTAG4") return EImageDescriberType::SIFT_CCTAG4;
#endif //HAVE_CCTAG
  if(imageDescriberType == "UNKNOWN") return EImageDescriberType::UNKNOWN;
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
