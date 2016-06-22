#include "SIFT_float_describer.hpp"

extern "C" {
#include "nonFree/sift/vl/sift.h"
}

namespace openMVG {
namespace features {

SIFT_float_describer::SIFT_float_describer(const SiftParams & params, bool bOrientation)
  : Image_describer()
  , _params(params)
  , _bOrientation(bOrientation)
{
  // Configure VLFeat
  vl_constructor();
}

SIFT_float_describer::~SIFT_float_describer()
{
  vl_destructor();
}

bool SIFT_float_describer::Describe(const image::Image<unsigned char>& image,
  std::unique_ptr<Regions> &regions,
  const image::Image<unsigned char> * mask)
{
  return extractSIFT<float>(image, regions, _params, _bOrientation, mask);
}

}
}
