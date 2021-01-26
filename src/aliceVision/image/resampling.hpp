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
   ** Half sample an image (ie reduce its size by a factor 2) using bilinear interpolation
   ** @param src input image
   ** @param out output image
   **/
  template < typename Image >
  void ImageHalfSample( const Image & src , Image & out )
  {
    const int new_width  = src.Width() / 2 ;
    const int new_height = src.Height() / 2 ;

    out.resize( new_width , new_height ) ;

    const Sampler2d<SamplerLinear> sampler;

    for( int i = 0 ; i < new_height ; ++i )
    {
      for( int j = 0 ; j < new_width ; ++j )
      {
        // Use .5f offset to ensure mid pixel and correct bilinear sampling
        out( i , j ) =  sampler( src, 2.f * (i+.5f), 2.f * (j+.5f) );
      }
    }
  }

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

  template <typename ImageType>
  void downscaleImageInplace(ImageType& inout, int downscale)
  {
      if(downscale <= 1)
          return;
      ALICEVISION_LOG_TRACE("downscaleImageInplace in: " << inout.Width() << "x" << inout.Height());
      /*
      {
          // Gaussian filter + Nearest Neighbor: could be expensive for very large resolutions with large downscale factor.
          Image otherImg;
          const double sigma = downscale * 0.5;
          ImageGaussianFilter(inout, sigma, otherImg); // TODO: error on RGBColor
          downscaleImage<SamplerNearest>(otherImg, inout, downscale);
          ALICEVISION_LOG_INFO("downscaleImageInplace otherImg: " << otherImg.Width() << "x" << otherImg.Height());
      }
      {
          // Simple bilinear interpolation: only valid for <= 2x downscale
          ImageType otherImg;
          downscaleImage<SamplerLinear>(inout, otherImg, downscale);
          std::swap(inout, otherImg);
      }*/
      {
          // Rely on OpenImageIO to do the downscaling
          const unsigned int w = inout.Width();
          const unsigned int h = inout.Height();
          const unsigned int nchannels = inout.Channels();

          const unsigned int nw = (unsigned int)(floor(float(w) / downscale));
          const unsigned int nh = (unsigned int)(floor(float(h) / downscale));

          ImageType rescaled(nw, nh);

          const oiio::ImageSpec imageSpecOrigin(w, h, nchannels, ColorTypeInfo<typename ImageType::Tpixel>::typeDesc);
          const oiio::ImageSpec imageSpecResized(nw, nh, nchannels, ColorTypeInfo<typename ImageType::Tpixel>::typeDesc);

          const oiio::ImageBuf inBuf(imageSpecOrigin, inout.data());
          oiio::ImageBuf outBuf(imageSpecResized, rescaled.data());

          oiio::ImageBufAlgo::resize(outBuf, inBuf);

          inout.swap(rescaled);
      }
      ALICEVISION_LOG_TRACE("downscaleImageInplace out: " << inout.Width() << "x" << inout.Height());
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
