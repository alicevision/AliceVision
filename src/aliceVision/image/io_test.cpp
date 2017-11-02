// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cstdio>
#include <iostream>
#include <string>

#include <aliceVision/system/Logger.hpp>
#include "aliceVision/image/image.hpp"

#define BOOST_TEST_MODULE imageIO
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::image;
using std::string;

BOOST_AUTO_TEST_CASE(ReadJpg_Jpg_Color) {
  Image<RGBColor> image;
  const std::string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_color.jpg";
  BOOST_CHECK(ReadImage(jpg_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(3, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), RGBColor(255, 125, 11));
  BOOST_CHECK_EQUAL(image(0,1), RGBColor( 20, 127, 255));
}

BOOST_AUTO_TEST_CASE(ReadJpg_Jpg_Monochrome) {
  Image<unsigned char> image;
  const std::string jpg_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_monochrome.jpg";
  BOOST_CHECK(ReadImage(jpg_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)0);
}

BOOST_AUTO_TEST_CASE(ReadPng_Png_Color) {
  Image<RGBAColor> image;
  const std::string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_color.png";
  BOOST_CHECK(ReadImage(png_filename.c_str(), &image));
  // Depth is 4 (RGBA by default)
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(4, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), RGBAColor(255, 125, 10, 255));
  BOOST_CHECK_EQUAL(image(0,1), RGBAColor( 20, 127, 255,255));
}

BOOST_AUTO_TEST_CASE(ReadPng_Png_Monochrome) {
  Image<unsigned char> image;
  const std::string png_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_monochrome.png";
  BOOST_CHECK(ReadImage(png_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)0);
}

BOOST_AUTO_TEST_CASE(GetFormat_filenames) {
  BOOST_CHECK_EQUAL(GetFormat("something.jpg"), aliceVision::image::Jpg);
  BOOST_CHECK_EQUAL(GetFormat("something.png"), aliceVision::image::Png);
  BOOST_CHECK_EQUAL(GetFormat("something.pnm"), aliceVision::image::Pnm);
  BOOST_CHECK_EQUAL(GetFormat("something.tif"), aliceVision::image::Tiff);
  BOOST_CHECK_EQUAL(GetFormat("/some/thing.JpG"), aliceVision::image::Jpg);
  BOOST_CHECK_EQUAL(GetFormat("/some/thing.pNG"), aliceVision::image::Png);
  BOOST_CHECK_EQUAL(GetFormat("some/thing.PNm"), aliceVision::image::Pnm);
  BOOST_CHECK_EQUAL(GetFormat("some/thing.TIf"), aliceVision::image::Tiff);
  BOOST_CHECK_EQUAL(GetFormat(".s/o.m/e.t/h.i/n.g.JPG"), aliceVision::image::Jpg);
  BOOST_CHECK_EQUAL(GetFormat(".s/o.m/e.t/h.i/n.g.PNG"), aliceVision::image::Png);
  BOOST_CHECK_EQUAL(GetFormat(".s/o.m/e.t/h.i/n.g.PNM"), aliceVision::image::Pnm);
  BOOST_CHECK_EQUAL(GetFormat(".s/o.m/e.t/h.i/n.g.TIF"), aliceVision::image::Tiff);
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Png_Out) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  const std::string out_filename = ("test_write_png.png");
  BOOST_CHECK(WriteImage(out_filename.c_str(), image));

  Image<unsigned char> read_image;
  BOOST_CHECK(ReadImage(out_filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(out_filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Png_Out_Color) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor(255,127,0);
  image(1,0) = RGBColor(0,127,255);
  const std::string out_filename = ("test_write_png_color.png");
  BOOST_CHECK(WriteImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  BOOST_CHECK(ReadImage(out_filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(out_filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_InvalidFiles) {
  Image<unsigned char> image;
  const std::string filename = string(THIS_SOURCE_DIR) + "/donotexist.jpg";
  BOOST_CHECK(!ReadImage(filename.c_str(), &image));
  BOOST_CHECK(!ReadImage("hopefully_unexisting_file", &image));
  remove(filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Jpg) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  const std::string filename = ("test_write_jpg.jpg");
  BOOST_CHECK(WriteJpg(filename.c_str(), image, 100));

  Image<unsigned char> read_image;
  BOOST_CHECK(ReadImage(filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(filename.c_str());
}

BOOST_AUTO_TEST_CASE(ReadPnm_Pgm) {
  Image<unsigned char> image;
  const std::string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.pgm";
  BOOST_CHECK(ReadImage(pgm_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)0);
}

BOOST_AUTO_TEST_CASE(ReadPnm_PgmComments) {
  Image<unsigned char> image;
  const std::string pgm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels_gray.pgm";
  BOOST_CHECK(ReadImage(pgm_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(1, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), (unsigned char)255);
  BOOST_CHECK_EQUAL(image(0,1), (unsigned char)0);
}


BOOST_AUTO_TEST_CASE(ImageIOTest_Pgm) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  const std::string out_filename = "test_write_pnm.pgm";
  BOOST_CHECK(WriteImage(out_filename.c_str(),image));

  Image<unsigned char> read_image;
  BOOST_CHECK(ReadImage(out_filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(out_filename.c_str());
}

BOOST_AUTO_TEST_CASE(ReadPnm_Ppm) {
  Image<RGBColor> image;
  const std::string ppm_filename = string(THIS_SOURCE_DIR) + "/image_test/two_pixels.ppm";
  BOOST_CHECK(ReadImage(ppm_filename.c_str(), &image));
  BOOST_CHECK_EQUAL(2, image.Width());
  BOOST_CHECK_EQUAL(1, image.Height());
  BOOST_CHECK_EQUAL(3, image.Depth());
  BOOST_CHECK_EQUAL(image(0,0), RGBColor( (unsigned char)255));
  BOOST_CHECK_EQUAL(image(0,1), RGBColor( (unsigned char)0));
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Ppm) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor((unsigned char)255);
  image(1,0) = RGBColor((unsigned char)0);
  const std::string out_filename = "test_write_pnm.ppm";
  BOOST_CHECK(WriteImage(out_filename.c_str(), image));

  Image<RGBColor> read_image;
  BOOST_CHECK(ReadImage(out_filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(out_filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Tiff_Gray) {
  Image<unsigned char> image(1,2);
  image(0,0) = 255;
  image(1,0) = 0;
  const std::string filename = ("test_write_tiff.tif");
  BOOST_CHECK(WriteImage(filename.c_str(), image));

  Image<unsigned char> read_image;
  BOOST_CHECK(ReadImage(filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Tiff_RGB) {
  Image<RGBColor> image(1,2);
  image(0,0) = RGBColor((unsigned char)255);
  image(1,0) = RGBColor((unsigned char)0);
  const std::string filename = ("test_write_tiff.tif");
  BOOST_CHECK(WriteImage(filename.c_str(), image));

  Image<RGBColor> read_image;
  BOOST_CHECK(ReadImage(filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageIOTest_Tiff_RGBA) {
  Image<RGBAColor> image(1,2);
  image(0,0) = RGBAColor(255, 125, 10, 255);
  image(1,0) = RGBAColor(2, 3, 4, 255);
  const std::string filename = ("test_write_tiff.tif");
  BOOST_CHECK(WriteImage(filename.c_str(), image));

  Image<RGBAColor> read_image;
  BOOST_CHECK(ReadImage(filename.c_str(), &read_image));
  BOOST_CHECK(read_image == image);
  remove(filename.c_str());
}

BOOST_AUTO_TEST_CASE(ImageHeader_AllFormats) {

  const std::vector<std::string> ext_Type = {"jpg", "png", "tif", "png", "pgm"};
  const int image_border_size = 10;
  for (int i=0; i < ext_Type.size(); ++i)
  {
    std::ostringstream os;
    os << "img" << "." << ext_Type[i];
    const std::string filename = os.str();
    ALICEVISION_LOG_DEBUG("Testing:" << filename);

    // Test for gray images
    {
      Image<unsigned char> gray_image(image_border_size, image_border_size);
      BOOST_CHECK(WriteImage(filename.c_str(), gray_image));
      ImageHeader imgHeader;
      BOOST_CHECK(ReadImageHeader(filename.c_str(), &imgHeader));
      BOOST_CHECK_EQUAL(image_border_size, imgHeader.width);
      BOOST_CHECK_EQUAL(image_border_size, imgHeader.height);
      remove(filename.c_str());
    }

    // Test for RGB images
    {
      Image<RGBColor> rgb_image(image_border_size, image_border_size);
      ImageHeader imgHeader;
      BOOST_CHECK(WriteImage(filename.c_str(), rgb_image));
      BOOST_CHECK(ReadImageHeader(filename.c_str(), &imgHeader));
      BOOST_CHECK_EQUAL(image_border_size, imgHeader.width);
      BOOST_CHECK_EQUAL(image_border_size, imgHeader.height);
      remove(filename.c_str());
    }
  }
}
