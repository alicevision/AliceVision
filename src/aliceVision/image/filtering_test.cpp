// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/image.hpp"

#include <iostream>

#define BOOST_TEST_MODULE ImageFiltering
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace std;

BOOST_AUTO_TEST_CASE(Image_Convolution)
{
  Image<unsigned char> in(250,250);
  for( int i = 10; i < 250-10; i++)
    for( int j = 10; j < 250-10; j++)
    {
      in(j,i) = rand()%127+127;
    }

  BOOST_CHECK(in(5,5) == 0);
  BOOST_CHECK(in(249-5,249-5) == 0);

  // filter
  Image<unsigned char> outFiltered(250,250);
  ImageGaussianFilter( in, 6.0, outFiltered);

  BOOST_CHECK(WriteImage("in.png", in));
  BOOST_CHECK(WriteImage("outfilter.png", outFiltered));
    
  // Check that gaussian filtering have smooth at border of the white random square
  BOOST_CHECK(outFiltered(5,5)>0);
  BOOST_CHECK(outFiltered(250-5,250-5)>0);
}

BOOST_AUTO_TEST_CASE(Image_Convolution_MeanBoxFilter_Separable)
{
  Image<unsigned char> in(40,40);
  in.block(10,10,20,20).fill(255.f);
  Vec3 meanBoxFilterKernel(1.f/3.f, 1.f/3.f, 1.f/3.f);
  Image<unsigned char> out;
  ImageSeparableConvolution( in , meanBoxFilterKernel, meanBoxFilterKernel , out);
}

BOOST_AUTO_TEST_CASE(Image_Convolution_MeanBoxFilter)
{
  Image<unsigned char> in(40,40);
  in.block(10,10,20,20).fill(255.f);
  Mat3 meanBoxFilterKernel;
  meanBoxFilterKernel.fill(1.f/9.f);
  Image<unsigned char> out;
  ImageConvolution(in, meanBoxFilterKernel, out);
}

BOOST_AUTO_TEST_CASE(Image_Convolution_Scharr_X_Y)
{
  Image<float> in(40,40);
  in.block(10,10,20,20).fill(255.f);

  Image<float> outFiltered(40,40);

  ImageScaledScharrXDerivative( in, outFiltered, 1);

  // X dir
  BOOST_CHECK_EQUAL(127.5f, outFiltered(20,10));
  BOOST_CHECK_EQUAL(-127.5f, outFiltered(20,30));
  // Y dir
  BOOST_CHECK_EQUAL(0.f, outFiltered(10,20));
  BOOST_CHECK_EQUAL(0.f, outFiltered(30,20));
  // Check it exist a vertical black band
  BOOST_CHECK_EQUAL(0.f, outFiltered.block(0,10+3,40,20-2*3).array().abs().sum());
  
  BOOST_CHECK(WriteImage("in_Scharr.png", Image<unsigned char>(in.cast<unsigned char>())));
  BOOST_CHECK(WriteImage("out_ScharrX.png", Image<unsigned char>(outFiltered.cast<unsigned char>())));

  outFiltered.fill(0.0f);
  ImageScaledScharrYDerivative( in, outFiltered, 1);

  // X dir
  BOOST_CHECK_EQUAL(0.f, outFiltered(20,10));
  BOOST_CHECK_EQUAL(0.f, outFiltered(20,30));
  // Y dir
  BOOST_CHECK_EQUAL(127.5f, outFiltered(10,20));
  BOOST_CHECK_EQUAL(-127.5f, outFiltered(30,20));
  // Check it exist a horizontal black band
  BOOST_CHECK_EQUAL(0.f, outFiltered.block(10+3,0,20-2*3,40).array().abs().sum());
  BOOST_CHECK(WriteImage("out_ScharrY.png", Image<unsigned char>(outFiltered.cast<unsigned char>())));
}

BOOST_AUTO_TEST_CASE(Image_Convolution_Sobel_X_Y)
{
  Image<float> in(40,40);
  in.block(10,10,20,20).fill(255.f);

  Image<float> outFiltered(40,40);

  ImageSobelXDerivative( in, outFiltered);

  // X dir
  BOOST_CHECK_EQUAL(127.5f, outFiltered(20,10));
  BOOST_CHECK_EQUAL(-127.5f, outFiltered(20,30));
  // Y dir
  BOOST_CHECK_EQUAL(0.f, outFiltered(10,20));
  BOOST_CHECK_EQUAL(0.f, outFiltered(30,20));
  // Check it exist a vertical black band
  BOOST_CHECK_EQUAL(0.f, outFiltered.block(0,10+3,40,20-2*3).array().abs().sum());

  BOOST_CHECK(WriteImage("in_Scharr.png", Image<unsigned char>(in.cast<unsigned char>())));
  BOOST_CHECK(WriteImage("out_SobelX.png", Image<unsigned char>(outFiltered.cast<unsigned char>())));

  outFiltered.fill(0.0f);
  ImageSobelYDerivative( in, outFiltered);

  // X dir
  BOOST_CHECK_EQUAL(0.f, outFiltered(20,10));
  BOOST_CHECK_EQUAL(0.f, outFiltered(20,30));
  // Y dir
  BOOST_CHECK_EQUAL(127.5f, outFiltered(10,20));
  BOOST_CHECK_EQUAL(-127.5f, outFiltered(30,20));
  // Check it exist a horizontal black band
  BOOST_CHECK_EQUAL(0.f, outFiltered.block(10+3,0,20-2*3,40).array().abs().sum());

  BOOST_CHECK(WriteImage("out_SobelY.png", Image<unsigned char>(outFiltered.cast<unsigned char>())));
}
