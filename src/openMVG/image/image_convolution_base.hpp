// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_IMAGE_IMAGE_CONVOLUTION_BASE_HPP
#define OPENMVG_IMAGE_IMAGE_CONVOLUTION_BASE_HPP

namespace openMVG {
namespace image {
  /**
   ** Filter an extended row [halfKernelSize][row][halfKernelSize]
   ** @param buffer data to filter
   ** @param kernel kernel array
   ** @param rsize buffer length
   ** @param ksize kernel length
  **/
  template<class T1, class T2> inline
  void conv_buffer_( T1* buffer, const T2* kernel, int rsize, int ksize )
  {
    for ( size_t i = 0; i < rsize; ++i )
    {
      T2 sum( 0 );
      for ( size_t j = 0; j < ksize; ++j )
      {
        sum += buffer[i + j] * kernel[j];
      }
      buffer[i] = sum;
    }
  }
} // namespace image
} // namespace openMVG

#endif // OPENMVG_IMAGE_IMAGE_CONVOLUTION_BASE_HPP
