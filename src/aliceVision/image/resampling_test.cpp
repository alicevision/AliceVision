// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/all.hpp>

#include <string>
#include <sstream>

#define BOOST_TEST_MODULE ImageRessampling

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::image;

BOOST_AUTO_TEST_CASE(Ressampling_SampleSamePosition)
{
  Image<unsigned char> image;
  std::string png_filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
  ALICEVISION_LOG_DEBUG(png_filename);
  BOOST_CHECK_NO_THROW(readImage(png_filename, image, image::EImageColorSpace::NO_CONVERSION));


  // Build sampling grid
  std::vector< std::pair< float , float > > sampling_grid ;
  for( int i = 0 ; i < image.Height() ; ++i )
  {
    for( int j = 0 ; j < image.Width() ; ++j )
    {
      sampling_grid.push_back( std::make_pair( i , j ) ) ;
    }
  }

  // Ressample image
  Sampler2d< SamplerLinear > sampler ;

  Image<unsigned char> imageOut ;

  GenericRessample( image , sampling_grid , image.Width() , image.Height() , sampler , imageOut ) ;

  std::string out_filename = ("test_ressample_same.png");
  BOOST_CHECK_NO_THROW( writeImage( out_filename, imageOut, image::EImageColorSpace::NO_CONVERSION) ) ;
}

// Iterative image rotations
// Allow to check if the sampling function have some signal loss.
template<typename SamplerT, typename ImageT>
void ImageRotation(
  const ImageT & imageIn,
  const SamplerT & sampler,
  const std::string & samplerString)
{
  bool bOk = true;
  ImageT image = imageIn;

  const int nb_rot = 6 ;
  const float delta = ( 2.0 * 3.141592 ) / nb_rot ;

  const float middle_x = image.Width() / 2.0 ;
  const float middle_y = image.Height() / 2.0 ; 

  // Rotate image then set starting image as source 
  for( int id_rot = 0 ; id_rot < nb_rot ; ++id_rot )
  {
    // angle of rotation (negative because it's inverse transformation)
    const float cur_angle = delta ;

    const float cs = cosf( -cur_angle ) ;
    const float ss = sinf( -cur_angle ) ; 

    std::vector< std::pair<float,float> > sampling_grid;
    // Compute sampling grid
    for( int i = 0 ; i < image.Height() ; ++i )
    {
      for( int j = 0 ; j < image.Width() ; ++j )
      {
        // Compute rotation of pixel (i,j) around center of image

        // Center pixel
        const float dx = static_cast<float>(j) - middle_x ;
        const float dy = static_cast<float>(i) - middle_y ;

        const float rotated_x = cs * dx - ss * dy ;
        const float rotated_y = ss * dx + cs * dy ;

        // Get back to original center
        const float cur_x = rotated_x + middle_x ;
        const float cur_y = rotated_y + middle_y ;

        sampling_grid.push_back( std::make_pair( cur_y , cur_x ) ) ;
      }
    }

    // Sample input image
    ImageT imageOut ;

    GenericRessample( image , sampling_grid , image.Width() , image.Height() , sampler , imageOut ) ;

    std::stringstream str ;
    str << "test_ressample_"<< samplerString <<"_rotate_" << id_rot << ".png" ;
    writeImage(str.str(), imageOut, image::EImageColorSpace::NO_CONVERSION);
    image = imageOut ;
  }
}

BOOST_AUTO_TEST_CASE(Ressampling_SampleRotate)
{
  Image<RGBColor> image;

  std::string png_filename = std::string(THIS_SOURCE_DIR) + "/image_test/lena.png";
  ALICEVISION_LOG_DEBUG(png_filename);
  BOOST_CHECK_NO_THROW(readImage(png_filename, image, image::EImageColorSpace::NO_CONVERSION));

  BOOST_CHECK_NO_THROW(ImageRotation(image, Sampler2d< SamplerNearest >(), "SamplerNearest"));
  BOOST_CHECK_NO_THROW(ImageRotation(image, Sampler2d< SamplerLinear >(), "SamplerLinear"));
  BOOST_CHECK_NO_THROW(ImageRotation(image, Sampler2d< SamplerCubic >(), "SamplerCubic"));
  BOOST_CHECK_NO_THROW(ImageRotation(image, Sampler2d< SamplerSpline16 >(), "SamplerSpline16"));
  BOOST_CHECK_NO_THROW(ImageRotation(image, Sampler2d< SamplerSpline64 >(), "SamplerSpline64"));
}
