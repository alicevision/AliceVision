#include "ASIFT_describer.hpp"

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

namespace openMVG {
namespace features {

ASIFT_Image_describer::ASIFT_Image_describer(const ASiftParams & params, bool bOrientation)
  : Image_describer()
  , _params(params)
  , _bOrientation(bOrientation)
{
  // Configure VLFeat
  vl_constructor();
}

ASIFT_Image_describer::~ASIFT_Image_describer()
{
  vl_destructor();
}

bool ASIFT_Image_describer::Describe(const image::Image<unsigned char>& image,
  std::unique_ptr<Regions> &regions,
  const image::Image<unsigned char> * mask)
{
  return extractASIFT<unsigned char>(image, regions, _params, _bOrientation, mask);
}

}
}
