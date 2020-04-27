// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/image/all.hpp"

#define BOOST_TEST_MODULE ImageDrawing

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::image;

// Horizontal / Vertical scanlines
// Assert that pixels was drawn at the good place
BOOST_AUTO_TEST_CASE(ImageDrawing_Scanlines) {

  const int w = 10, h = 10;
  Image<unsigned char> image(h,w);
  image.fill(0);

  // horizontal scanline
  //  __________
  //  |         |
  //  |__here___|
  //  |         |
  //  |_________|
  const int y = 5;
  DrawLine( 0, y, w-1, y, 255, &image);
  for(int i=0; i < w; ++i)
    BOOST_CHECK_EQUAL( image(y,i), 255);

  image.fill(0);

  // Vertical scanline
  //  __________
  //  |    h|   |
  //  |    e|   |
  //  |    r|   |
  //  |____e|___|
  const int x = 5;
  DrawLine( x, 0, x, h-1, 255, &image);
  for (int i = 0; i < h; ++i)
    BOOST_CHECK_EQUAL(image(i,y), 255);
}

BOOST_AUTO_TEST_CASE(ImageDrawing_Scanlines_RGB) {

  const int w = 10, h = 10;
  Image<RGBColor> image(h,w);
  image.fill(RGBColor(BLACK));

  // horizontal scanline
  //  __________
  //  |         |
  //  |__here___|
  //  |         |
  //  |_________|
  const int y = 5;
  DrawLine( 0, y, w-1, y, RGBColor(GREEN), &image);
  for(int i=0; i < w; ++i)
    BOOST_CHECK_EQUAL( image(y,i), RGBColor(GREEN));

  image.fill(RGBColor(BLACK));

  // Vertical scanline
  //  __________
  //  |    h|   |
  //  |    e|   |
  //  |    r|   |
  //  |____e|___|
  const int x = 5;
  DrawLine( x, 0, x, h-1, RGBColor(YELLOW), &image);
  for (int i = 0; i < h; ++i)
    BOOST_CHECK_EQUAL(image(i,y), RGBColor(YELLOW));
}

// Lines with a given angle +/-45°
// Assert that pixels was drawn at the good place
BOOST_AUTO_TEST_CASE(ImageDrawing_Lines45) {

  const int w = 10, h = 10;
  Image<unsigned char> image(h,w);
  image.fill(0);

  //  _____
  //  |\  |
  //  | \ |
  //  |__\|

  DrawLine(0, 0, w-1, h-1, 255, &image);
  for (int i = 0; i < w; ++i)
    BOOST_CHECK_EQUAL(image(i,i), 255);

  image.fill(0);

  //  _____
  //  |  / |
  //  | /  |
  //  |/___|_
  DrawLine(0, h-1, w-1, 0, 255, &image);
  for (int i = 0; i < h; ++i)
    BOOST_CHECK_EQUAL(image(h-1-i,i), 255);
}

// Draw a circle in an image and assert that all the points are
// at a distance equal to the radius.
BOOST_AUTO_TEST_CASE(ImageDrawing_Circle) {

  Image<unsigned char> image(10,10);
  image.fill(0);

  const int radius = 3;
  const int x = 5, y = 5;

  DrawCircle(x, y, radius, (unsigned char)255, &image);

  // Distance checking :
  for ( int j = 0; j < image.Height(); ++j)
  for ( int i = 0; i < image.Width(); ++i) {
    if (image(j, i) == 255)  {
      const float distance =  std::hypot((float)(j-y), (float)(i-x));
      BOOST_CHECK_SMALL(radius-distance, 1.0f);
      // Due to discretisation we cannot expect better precision
    }
  }
}

// Draw an ellipse with the two radius equal each other...
// in an image and assert that all the points are
// at a distance equal to the radius.
BOOST_AUTO_TEST_CASE(ImageDrawing_Ellipse) {

  Image<unsigned char> image(10,10);
  image.fill(0);

  const int radius = 3, angle = 0;
  const int x = 5, y = 5;

  DrawEllipse(x, y, radius, radius, (unsigned char)255, &image, (double)angle);

  // Distance checking :
  for(int j = 0; j < image.Height(); ++j)
  {
    for(int i = 0; i < image.Width(); ++i)
    {
      if(image(j, i) == 255)
      {
        const float distance = std::hypot((float) (j - y), (float) (i - x));
        BOOST_CHECK_SMALL(radius-distance, 1.0f);
        // Due to discretisation we cannot expect better precision
      }
    }
  }
}

// Draw an ellipse with the two radius and rotated ...
// in an image and assert that all the points are
// within the given radius.
BOOST_AUTO_TEST_CASE(ImageDrawing_RotatedEllipse) {

  Image<unsigned char> image(30,30);
  image.fill(0);

  const int radius = 6;
  const int x = 10, y = 10;

  DrawEllipse(x, y, radius, radius/2.0, static_cast<unsigned char>(255), &image, M_PI/4.0);

  // Distance checking :
  for(int j = 0; j < image.Height(); ++j)
  {
    for(int i = 0; i < image.Width(); ++i)
    {
      if(image(j, i) == 255)
      {
        const float distance = std::hypot((float) (j - y), (float) (i - x));
        BOOST_CHECK_EQUAL(radius + 1 >= distance && radius / 2.0 - 1 <= distance, true);
        // Due to discretization we cannot expect better precision
        // Use +-1 to avoid rasterization error.
      }
    }
  }
}

/// Assert that the DrawLine function do not crash
/// when one point is outside the image
BOOST_AUTO_TEST_CASE(ImageDrawing_DrawLine_PointOutsideTheImage) {

  Image<unsigned char> image(30,30);
  image.fill(0);

  const int radius = 20;
  int x = 15, y = 15;

  // Distance checking :
  for (double i=0; i < 2.0*3.14; i+=3.14/12)
  {
    int x1 = int(cos(i) * radius + 0.5);
    int y1 = int(sin(i) * radius + 0.5);
    DrawLine( x, y, x+x1, y+y1, 255, &image);
  }
  // Translate :
  x += int(15/2.0+0.5);
  for (double i=0; i < 2.0*3.14; i+=3.14/12)
  {
    int x1 = int(cos(i) * radius + 0.5);
    int y1 = int(sin(i) * radius + 0.5);
    DrawLine( x, y, x+x1, y+y1, 255, &image);
  }
  // Translate :
  x += int(15/2.0+0.5);
  for (double i=0; i < 2.0*3.14; i+=3.14/12)
  {
    int x1 = int(cos(i) * radius + 0.5);
    int y1 = int(sin(i) * radius + 0.5);
    DrawLine( x, y, x+x1, y+y1, 255, &image);
  }

  //Point totally outside the image
  x = y = -100;
  for (double i=0; i < 2.0*3.14; i+=3.14/12)
  {
    int x1 = int(cos(i) * radius + 0.5);
    int y1 = int(sin(i) * radius + 0.5);
    DrawLine( x, y, x+x1, y+y1, 255, &image);
  }
  //writeImage( image, "toto.png");
}
