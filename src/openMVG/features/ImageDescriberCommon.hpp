#pragma once

#include "openMVG/features/image_describer.hpp"

#include <string>
#include <memory>

namespace openMVG {
namespace features {

enum class EImageDescriberType
{
  SIFT,             
  SIFT_FLOAT,        
#ifdef HAVE_CCTAG
  CCTAG3,            
  CCTAG4,
  SIFT_CCTAG3,
  SIFT_CCTAG4,
#endif //HAVE_CCTAG
  AKAZE_FLOAT,
  AKAZE_MLDB
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


 // Create the desired Image_describer method.
 // Don't use a factory, perform direct allocation
std::unique_ptr<Image_describer> createImageDescriber(EImageDescriberType imageDescriberType);

  
} // namespace features
} // namespace openMVG