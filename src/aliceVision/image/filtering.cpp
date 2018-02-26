// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "filtering.hpp"

namespace aliceVision {
namespace image {

Vec ComputeGaussianKernel(const std::size_t size, const double sigma)
{
  // If kernel size is 0 computes it's size using uber formula
  std::size_t k_size = ( size == 0 ) ? ceil(2.0*(1.0 + (sigma-0.8)/(0.3))) : size ;

  // Make kernel odd width
  k_size = ( k_size % 2 == 0 ) ? k_size + 1 : k_size ;
  const std::size_t half_k_size = ( k_size - 1 ) / 2 ;

  Vec res( k_size ) ;

  const double exp_scale = 1.0 / ( 2.0 * sigma * sigma ) ;

  // Compute unnormalized kernel
  double sum = 0.0 ;
  for( std::size_t i = 0 ; i < k_size ; ++i )
  {
    const double dx = ( static_cast<double>(i) - static_cast<double>(half_k_size) ) ;
    res( i ) = exp( - dx * dx * exp_scale ) ;
    sum += res( i ) ;
  }

  // Normalize kernel
  const double inv = 1.0 / sum ;
  for( std::size_t i = 0 ; i < k_size ; ++i )
  {
    res( i ) *= inv ;
  }

  return res ;
}

} // namespace image
} // namespace aliceVision
