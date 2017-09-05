// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_IMAGE_IMAGE_CONCAT_H_
#define OPENMVG_IMAGE_IMAGE_CONCAT_H_

#include "openMVG/image/image_container.hpp"

namespace openMVG {
namespace image {

/// Horizontal concatenation of images
template < class Image >
void ConcatH(const Image & imageA, const Image & imageB, Image & Out)
{
  // Compute new dimensions // |imgA|+|imgB|
  int ww = imageA.Width() + imageB.Width();
  Out.resize(ww, std::max(imageA.Height(), imageB.Height()));

  // Copy the first image |imgA|...|
  Out.block(0,0, imageA.Height(), imageA.Width()) = imageA.GetMat();
  // Copy the second image |imgA|imgB|
  Out.block(0, imageA.Width(), imageB.Height(), imageB.Width()) = imageB.GetMat();
}

/// Vertical concatenation of images
template < class Image >
void ConcatV(const Image & imageA, const Image & imageB, Image & Out)
{
  // Compute new dimensions
  // |imgA|
  // |imgB|
  int hh = imageA.Height() + imageB.Height();
  Out.resize(max(imageA.Width(), imageB.Width()), hh);

  // Copy the first image
  // |imgA|
  // |....|
  Out.block(0,0, imageA.Height(), imageA.Width()) = imageA.GetMat();
  // Copy the second image
  // |imgA|
  // |imgB|
  Out.block(imageA.Height(), 0, imageB.Height(), imageB.Width()) = imageB.GetMat();
}

} // namespace image
} // namespace openMVG

#endif // OPENMVG_IMAGE_IMAGE_CONCAT_H_
