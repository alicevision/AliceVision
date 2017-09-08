// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_CAMERA_UNDISTORT_IMAGE_HPP
#define OPENMVG_CAMERA_UNDISTORT_IMAGE_HPP

#include "aliceVision/image/image.hpp"
#include <aliceVision/config.hpp>

namespace aliceVision {
namespace cameras {

/// Undistort an image according a given camera & it's distortion model
template <typename Image>
void UndistortImage(
  const Image& imageIn,
  const IntrinsicBase * cam,
  Image & image_ud,
  typename Image::Tpixel fillcolor = typename Image::Tpixel(0))
{
  if (!cam->have_disto()) // no distortion, perform a direct copy
  {
    image_ud = imageIn;
  }
  else // There is distortion
  {
    image_ud.resize(imageIn.Width(), imageIn.Height(), true, fillcolor);
    const image::Sampler2d<image::SamplerLinear> sampler;

    #pragma omp parallel for
    for (int j = 0; j < imageIn.Height(); ++j)
      for (int i = 0; i < imageIn.Width(); ++i)
      {
        const Vec2 undisto_pix(i,j);
        // compute coordinates with distortion
        const Vec2 disto_pix = cam->get_d_pixel(undisto_pix);
        // pick pixel if it is in the image domain
        if ( imageIn.Contains(disto_pix(1), disto_pix(0)) )
          image_ud( j, i ) = sampler(imageIn, disto_pix(1), disto_pix(0));
      }
  }
}

} // namespace cameras
} // namespace aliceVision

#endif // #ifndef OPENMVG_CAMERA_UNDISTORT_IMAGE_HPP

