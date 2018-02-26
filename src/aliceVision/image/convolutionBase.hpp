// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstddef>

namespace aliceVision {
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
    for( std::size_t i = 0; i < rsize; ++i )
    {
      T2 sum( 0 );
      for ( std::size_t j = 0; j < ksize; ++j )
      {
        sum += buffer[i + j] * kernel[j];
      }
      buffer[i] = sum;
    }
  }
} // namespace image
} // namespace aliceVision
