// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/image/all.hpp"

#include <iostream>

#define BOOST_TEST_MODULE Image

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;

BOOST_AUTO_TEST_CASE(Image_Basis)
{
  //-- Gray(unsigned char) Image creation
  Image<unsigned char> imaGray(10,10);
  imaGray(1,1) = 1; //-- Pixel modification
  imaGray(2,2) = 2;
  imaGray(5,0) = 2;

  //cout << imaGray << endl << endl;
  //-- Get raw ptr to image data :
  const unsigned char * ptr = imaGray.data();
  ((unsigned char*)ptr)[0] = 2;
  fill(((unsigned char*)ptr+9*10),((unsigned char*)ptr+10*10),2);
  //cout << "After" << endl << imaGray;

  // Construction by re-copy
  Image<unsigned char> imageGray2(imaGray);

  // Construction by matrix
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix(5,5);
  Image<unsigned char> imageGray3 = matrix;

  //- Get back matrix contained in the image
  matrix = imaGray.GetMat();

  // clone of a matrix
  Image<unsigned char> imageGray4;
  imageGray4 = matrix;

  Image<unsigned char> imageGray5;
  imageGray5 = imaGray;

  //-- RGB Image creation
  Image<RGBColor> imaRGB(10,10);
  imaRGB(0,0) = RGBColor(0,1,2);

  //-- RGBA Image creation
  Image<RGBAColor> imaRGBA(10,10);
  imaRGBA(0,0) = RGBAColor(0,1,2,1);
  imaRGBA(1,0) = RGBAColor(1,1,1);

  // Image resizing
  Image<unsigned char> imaToResize;
  imaToResize.resize(5,10);
  BOOST_CHECK_EQUAL(10, imaToResize.Height());
  BOOST_CHECK_EQUAL(5, imaToResize.Width());
}

BOOST_AUTO_TEST_CASE(Image_PixelTypes)
{
  RGBColor  a(BLACK);
  // RGBColor  c(0); // Not accepted because can cause bad pixel affectation value (mixed type...)
  // The following issue must used : (at your own risk)
  RGBColor  b(static_cast<unsigned char>(0));
  RGBAColor d(BLACK, 255);
}

BOOST_AUTO_TEST_CASE(Image_ImageConverter)
{
  Image<RGBColor> imaColorRGB(5,5);
  imaColorRGB.fill(RGBColor(10,10,10));
  Image<unsigned char> imaGray;
  ConvertPixelType(imaColorRGB, &imaGray);

  //RGBA
  Image<RGBAColor> imaColorRGBA(5,5);
  imaColorRGBA.fill(RGBAColor(10,10,10, 255));
  ConvertPixelType(imaColorRGBA, &imaGray);
}
