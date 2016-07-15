#include "SIFT_describer.hpp"

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

namespace openMVG {
namespace features {

SIFT_Image_describer::SIFT_Image_describer(const SiftParams & params, bool bOrientation)
  : Image_describer()
  , _params(params)
  , _bOrientation(bOrientation)
{
  // Configure VLFeat
  vl_constructor();
}

SIFT_Image_describer::~SIFT_Image_describer()
{
  vl_destructor();
}

bool SIFT_Image_describer::Describe(const image::Image<unsigned char>& image,
  std::unique_ptr<Regions> &regions,
  const image::Image<unsigned char> * mask)
{
  return extractSIFT<unsigned char>(image, regions, _params, _bOrientation, mask);
}

}
}