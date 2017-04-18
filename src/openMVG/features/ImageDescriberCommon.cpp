#include "ImageDescriberCommon.hpp"

#include "openMVG/types.hpp"
#include "openMVG/features/image_describer_akaze.hpp"
#include "openMVG/features/cctag/CCTAG_describer.hpp"
#include "openMVG/features/cctag/SIFT_CCTAG_describer.hpp"

#include "nonFree/sift/SIFT_describer.hpp"
#include "nonFree/sift/SIFT_float_describer.hpp"

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
    case EImageDescriberType::AKAZE_MLDB:  return "AKAZE_MLDB";
    
#ifdef HAVE_CCTAG
    case EImageDescriberType::CCTAG3:      return "CCTAG3";
    case EImageDescriberType::CCTAG4:      return "CCTAG4";
    case EImageDescriberType::SIFT_CCTAG3: return "SIFT_CCTAG3";
    case EImageDescriberType::SIFT_CCTAG4: return "SIFT_CCTAG4";
#endif //HAVE_CCTAG
  }
  throw std::out_of_range("Invalid imageDescriber enum");
}

EImageDescriberType EImageDescriberType_stringToEnum(const std::string& imageDescriberType)
{
  if(imageDescriberType == "SIFT")        return EImageDescriberType::SIFT;
  if(imageDescriberType == "SIFT_FLOAT")  return EImageDescriberType::SIFT_FLOAT;
  if(imageDescriberType == "AKAZE_FLOAT") return EImageDescriberType::AKAZE_FLOAT;
  if(imageDescriberType == "AKAZE_MLDB")  return EImageDescriberType::AKAZE_MLDB;
  
#ifdef HAVE_CCTAG
  if(imageDescriberType == "CCTAG3")      return EImageDescriberType::CCTAG3;
  if(imageDescriberType == "CCTAG4")      return EImageDescriberType::CCTAG4;
  if(imageDescriberType == "SIFT_CCTAG3") return EImageDescriberType::SIFT_CCTAG3;
  if(imageDescriberType == "SIFT_CCTAG4") return EImageDescriberType::SIFT_CCTAG4;
#endif //HAVE_CCTAG
  throw std::out_of_range("Invalid imageDescriber : " + imageDescriberType);
}


std::unique_ptr<Image_describer> createImageDescriber(EImageDescriberType imageDescriberType)
{
  std::unique_ptr<Image_describer> discriberPtr;
  
  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:        discriberPtr.reset(new SIFT_Image_describer(SiftParams())); break;
    case EImageDescriberType::SIFT_FLOAT:  discriberPtr.reset(new SIFT_float_describer(SiftParams())); break;
    case EImageDescriberType::AKAZE_FLOAT: discriberPtr.reset(new AKAZE_Image_describer(AKAZEParams(AKAZEConfig(), features::AKAZE_MSURF))); break;
    case EImageDescriberType::AKAZE_MLDB:  discriberPtr.reset(new AKAZE_Image_describer(AKAZEParams(AKAZEConfig(), features::AKAZE_MLDB))); break;
    
#ifdef HAVE_CCTAG
    case EImageDescriberType::CCTAG3:      discriberPtr.reset(new CCTAG_Image_describer(3)); break;
    case EImageDescriberType::CCTAG4:      discriberPtr.reset(new CCTAG_Image_describer(4)); break;
    case EImageDescriberType::SIFT_CCTAG3: discriberPtr.reset(new SIFT_CCTAG_Image_describer(SiftParams(), true, 3)); break;
    case EImageDescriberType::SIFT_CCTAG4: discriberPtr.reset(new SIFT_CCTAG_Image_describer(SiftParams(), true, 4)); break;
#endif //HAVE_CCTAG
    
    default: throw std::out_of_range("Invalid imageDescriber enum");
  }       
  assert(discriberPtr != nullptr);
  
  return discriberPtr;
}

} // namespace features
} // namespace openMVG