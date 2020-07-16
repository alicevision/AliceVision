// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include "aliceVision/image/all.hpp"

#define BOOST_TEST_MODULE imageIO

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include <cstdio>
#include <iostream>
#include <vector>
#include <string>

using namespace aliceVision;
using namespace aliceVision::image;
using std::string;

// tested extensions
static std::vector<std::string> extensions = {"jpg", "png", "pgm", "ppm", "tiff", "exr"};

BOOST_AUTO_TEST_CASE(read_unexisting) {
  Image<unsigned char> image;
  const std::string filename = string(THIS_SOURCE_DIR) + "/unexisting.jpg";
  BOOST_CHECK_THROW(readImage(filename, image, image::EImageColorSpace::NO_CONVERSION), std::exception);
}

BOOST_AUTO_TEST_CASE(read_jpg_grayscale) {
  Image<unsigned char> image;
  const std::string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_monochrome.jpg";
  BOOST_CHECK_NO_THROW(readImage(jpg_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)  0);
}

BOOST_AUTO_TEST_CASE(read_jpg_rgb) {
  Image<RGBColor> image;
  const std::string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_color.jpg";
  BOOST_CHECK_NO_THROW(readImage(jpg_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(3, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0).r(), 255);
  BOOST_CHECK_EQUAL(image(0,0).g(), 125);
  BOOST_CHECK_EQUAL(image(0,0).b(),  11);
  BOOST_CHECK_EQUAL(image(0,1).r(),  20);
  BOOST_CHECK_EQUAL(image(0,1).g(), 127);
  BOOST_CHECK_EQUAL(image(0,1).b(), 255);
}

BOOST_AUTO_TEST_CASE(read_png_grayscale) {
  Image<unsigned char> image;
  const std::string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_monochrome.png";
  BOOST_CHECK_NO_THROW(readImage(png_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)  0);
}

BOOST_AUTO_TEST_CASE(read_png_rgb) {
  Image<RGBColor> image;
  const std::string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_color.png";
  BOOST_CHECK_NO_THROW(readImage(png_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(3, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0).r(), 255);
  BOOST_CHECK_EQUAL(image(0,0).g(), 125);
  BOOST_CHECK_EQUAL(image(0,0).b(),  10);
  BOOST_CHECK_EQUAL(image(0,1).r(),  20);
  BOOST_CHECK_EQUAL(image(0,1).g(), 127);
  BOOST_CHECK_EQUAL(image(0,1).b(), 255);
}

BOOST_AUTO_TEST_CASE(read_png_rgba) {
  Image<RGBAColor> image;
  const std::string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_color.png";
  BOOST_CHECK_NO_THROW(readImage(png_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(4, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0).r(), 255);
  BOOST_CHECK_EQUAL(image(0,0).g(), 125);
  BOOST_CHECK_EQUAL(image(0,0).b(),  10);
  BOOST_CHECK_EQUAL(image(0,0).a(), 255);
  BOOST_CHECK_EQUAL(image(0,1).r(),  20);
  BOOST_CHECK_EQUAL(image(0,1).g(), 127);
  BOOST_CHECK_EQUAL(image(0,1).b(), 255);
  BOOST_CHECK_EQUAL(image(0,0).a(), 255);
}

BOOST_AUTO_TEST_CASE(read_pgm) {
  Image<unsigned char> image;
  const std::string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.pgm";
  BOOST_CHECK_NO_THROW(readImage(pgm_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)  0);
}

BOOST_AUTO_TEST_CASE(read_pgm_withcomments) {
  Image<unsigned char> image;
  const std::string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_gray.pgm";
  BOOST_CHECK_NO_THROW(readImage(pgm_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)  0);
}

BOOST_AUTO_TEST_CASE(read_ppm) {
  Image<RGBColor> image;
  const std::string ppm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.ppm";
  BOOST_CHECK_NO_THROW(readImage(ppm_filename, image, image::EImageColorSpace::NO_CONVERSION));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(3, image.Depth());

  BOOST_CHECK_EQUAL(image(0,0).r(), 255);
  BOOST_CHECK_EQUAL(image(0,0).g(), 255);
  BOOST_CHECK_EQUAL(image(0,0).b(), 255);
  BOOST_CHECK_EQUAL(image(0,1).r(),   0);
  BOOST_CHECK_EQUAL(image(0,1).g(),   0);
  BOOST_CHECK_EQUAL(image(0,1).b(),   0);
}

BOOST_AUTO_TEST_CASE(read_write_grayscale) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;

  for(const auto& extension : extensions)
  {
    const std::string filename = "test_write." + extension;
    BOOST_CHECK_NO_THROW(writeImage(filename, image, image::EImageColorSpace::NO_CONVERSION));

    Image<unsigned char> read_image;
    BOOST_CHECK_NO_THROW(readImage(filename, read_image, image::EImageColorSpace::NO_CONVERSION));
    BOOST_CHECK_EQUAL(read_image(0,0), image(0,0));
    BOOST_CHECK_EQUAL(read_image(1,0), image(1,0));
    remove(filename.c_str());
  }
}

BOOST_AUTO_TEST_CASE(read_write_rgb) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor(255,   0, 255);
  image(1,0) = RGBColor(  0, 255,   0);

  for(const auto& extension : extensions)
  {
    const std::string filename = "test_write_rgb." + extension;
    BOOST_CHECK_NO_THROW(writeImage(filename, image, image::EImageColorSpace::NO_CONVERSION));

    Image<RGBColor> read_image;
    BOOST_CHECK_NO_THROW(readImage(filename, read_image, image::EImageColorSpace::NO_CONVERSION));

    if(extension != "jpg")
    {
      // doesn't have compression
      BOOST_CHECK_EQUAL(read_image(0,0).r(), image(0,0).r());
      BOOST_CHECK_EQUAL(read_image(0,0).g(), image(0,0).g());
      BOOST_CHECK_EQUAL(read_image(0,0).b(), image(0,0).b());
      BOOST_CHECK_EQUAL(read_image(1,0).r(), image(1,0).r());
      BOOST_CHECK_EQUAL(read_image(1,0).g(), image(1,0).g());
      BOOST_CHECK_EQUAL(read_image(1,0).b(), image(1,0).b());
    }
    remove(filename.c_str());
  }
}

BOOST_AUTO_TEST_CASE(read_write_rgba) {
  Image<RGBAColor> image(1,2);
  image(0,0) = RGBAColor(255,   0, 255, 255);
  image(1,0) = RGBAColor(  0, 255,   0, 255);

  for(const auto& extension : extensions)
  {
    if(extension == "jpg" ||
       extension == "pgm" ||
       extension == "ppm")
      continue; // doesn't support 4 channels

    const std::string filename = "test_write_rgba." + extension;
    BOOST_CHECK_NO_THROW(writeImage(filename, image, image::EImageColorSpace::NO_CONVERSION));

    Image<RGBAColor> read_image;
    BOOST_CHECK_NO_THROW(readImage(filename, read_image, image::EImageColorSpace::NO_CONVERSION));
    BOOST_CHECK_EQUAL(read_image(0,0).r(), image(0,0).r());
    BOOST_CHECK_EQUAL(read_image(0,0).g(), image(0,0).g());
    BOOST_CHECK_EQUAL(read_image(0,0).b(), image(0,0).b());
    BOOST_CHECK_EQUAL(read_image(0,0).a(), image(0,0).a());
    BOOST_CHECK_EQUAL(read_image(1,0).r(), image(1,0).r());
    BOOST_CHECK_EQUAL(read_image(1,0).g(), image(1,0).g());
    BOOST_CHECK_EQUAL(read_image(1,0).b(), image(1,0).b());
    BOOST_CHECK_EQUAL(read_image(1,0).a(), image(1,0).a());
    remove(filename.c_str());
  }
}

BOOST_AUTO_TEST_CASE(read_write_from_rgb_to_grayscale) {
  Image<RGBColor> imageRGB(1,2);
  imageRGB(0,0) = RGBColor(  0,    0,    0);
  imageRGB(1,0) = RGBColor(255,  255,  255);

  Image<unsigned char> imageGrayscale(1,2);
  imageGrayscale(0,0) =   0;
  imageGrayscale(1,0) = 255;

  for(const auto& extension : extensions)
  {
    const std::string filename = "test_write_from_grayscale." + extension;
    BOOST_CHECK_NO_THROW(writeImage(filename, imageRGB, image::EImageColorSpace::NO_CONVERSION));

    Image<unsigned char> read_image;
    BOOST_CHECK_NO_THROW(readImage(filename, read_image, image::EImageColorSpace::NO_CONVERSION));
    //BOOST_CHECK_EQUAL(read_image, imageGrayscale);
    remove(filename.c_str());
  }
}

BOOST_AUTO_TEST_CASE(read_write_from_grayscale_to_rgb) {
  Image<RGBColor> imageRGB(1,2);
  imageRGB(0,0) = RGBColor(  0,    0,    0);
  imageRGB(1,0) = RGBColor(255,  255,  255);

  Image<unsigned char> imageGrayscale(1,2);
  imageGrayscale(0,0) =   0;
  imageGrayscale(1,0) = 255;

  for(const auto& extension : extensions)
  {
    const std::string filename = "test_write_from_rgb." + extension;
    BOOST_CHECK_NO_THROW(writeImage(filename, imageGrayscale, image::EImageColorSpace::NO_CONVERSION));

    Image<RGBColor> read_image;
    BOOST_CHECK_NO_THROW(readImage(filename, read_image, image::EImageColorSpace::NO_CONVERSION));
    BOOST_CHECK_EQUAL(read_image(0,0).r(), imageRGB(0,0).r());
    BOOST_CHECK_EQUAL(read_image(0,0).g(), imageRGB(0,0).g());
    BOOST_CHECK_EQUAL(read_image(0,0).b(), imageRGB(0,0).b());
    BOOST_CHECK_EQUAL(read_image(1,0).r(), imageRGB(1,0).r());
    BOOST_CHECK_EQUAL(read_image(1,0).g(), imageRGB(1,0).g());
    BOOST_CHECK_EQUAL(read_image(1,0).b(), imageRGB(1,0).b());
    remove(filename.c_str());
  }
}
