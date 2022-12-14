// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "io.hpp"

#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/system/Logger.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

namespace oiio = OIIO;

namespace aliceVision {
namespace image {
  
  /**
   * @brief Downscale an image using a given type of sampler.
   * @param[in] src source image to downscale
   * @param[out] out image to store the downscaled result
   * @param[in] downscale downscale value
   */
  template <typename SamplerType, typename Image>
  void downscaleImage(const Image& src, Image& out, int downscale)
  {
      const int new_width = src.Width() / downscale;
      const int new_height = src.Height() / downscale;

      out.resize(new_width, new_height);

      const Sampler2d<SamplerType> sampler;
      const float downscalef = downscale;
      for(int i = 0; i < new_height; ++i)
      {
          for(int j = 0; j < new_width; ++j)
          {
              // Use .5f offset to ensure mid pixel and correct sampling
              out(i, j) = sampler(src, downscalef * (i + .5f), downscalef * (j + .5f));
          }
      }
  }

  /**
   ** Half sample an image (ie reduce its size by a factor 2) using bilinear interpolation
   ** @param src input image
   ** @param out output image
   **/
  template < typename Image >
  void ImageHalfSample( const Image & src , Image & out )
  {
    downscaleImage<SamplerLinear, Image>(src, out, 2);
  }

  /**
   ** @brief Ressample an image using given sampling positions
   ** @param src Input image
   ** @param sampling_pos A list of coordinates where the image needs to be ressampled (samples are (Y,X) )
   ** @param output_width Width of the output image.
   ** @param output_height Height of the output image
   ** @param sampling_func Ressampling functor used to sample the Input image
   ** @param[out] Output image
   ** @note sampling_pos.size() must be equal to output_width * output_height
   **/
  template <typename Image , typename RessamplingFunctor>
  void GenericRessample( const Image & src ,
                         const std::vector< std::pair< float , float > > & sampling_pos ,
                         const int output_width ,
                         const int output_height ,
                         const RessamplingFunctor & sampling_func ,
                         Image & out )
  {
    assert( sampling_pos.size() == output_width * output_height );

    out.resize( output_width , output_height );

    std::vector< std::pair< float , float > >::const_iterator it_pos = sampling_pos.begin();

    for( int i = 0 ; i < output_height ; ++i )
    {
      for( int j = 0 ; j < output_width ; ++j , ++it_pos )
      {
        const float input_x = it_pos->second ;
        const float input_y = it_pos->first ;

        out( i , j ) = sampling_func( src , input_y , input_x ) ;
      }
    }
  }

} // namespace image
} // namespace aliceVision
