// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>


#include <boost/test/unit_test.hpp>

#define EXPECT_MATRIX_CLOSE_FRACTION(a, b, tolerance) \
{ \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  BOOST_CHECK_EQUAL(a.rows(),b.rows()); \
  BOOST_CHECK_EQUAL(a.cols(),b.cols()); \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        BOOST_CHECK_CLOSE_FRACTION(a(r, c), b(r, c), tolerance); \
      } \
    } \
  } \
}

#define EXPECT_MATRIX_NEAR(a, b, tolerance) \
{ \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  BOOST_CHECK_EQUAL(a.rows(),b.rows()); \
  BOOST_CHECK_EQUAL(a.cols(),b.cols()); \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        BOOST_CHECK_SMALL(a(r, c)-b(r, c), tolerance); \
      } \
    } \
  } \
}

#define EXPECT_MATRIX_NEAR_ZERO(a, tolerance) \
{ \
  for (int r = 0; r < a.rows(); ++r) { \
    for (int c = 0; c < a.cols(); ++c) { \
      BOOST_CHECK_SMALL(a(r, c), tolerance) \
    } \
  } \
}

#define EXPECT_MATRIX_EQ(a, b) \
{ \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  BOOST_CHECK_EQUAL(a.rows(),b.rows()); \
  BOOST_CHECK_EQUAL(a.cols(),b.cols()); \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        BOOST_CHECK_EQUAL(a(r, c), b(r, c)) \
      } \
    } \
  } \
}

#define EXPECT_EQ(a, b) BOOST_CHECK_EQUAL(a,b);

// Check that sin(angle(a, b)) < tolerance.
#define EXPECT_MATRIX_PROP(a, b, tolerance) \
{ \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  BOOST_CHECK_EQUAL(a.rows(), b.rows()); \
  BOOST_CHECK_EQUAL(a.cols(), b.cols()); \
  if (dims_match) { \
    double c = CosinusBetweenMatrices(a, b); \
    if (c * c < 1) { \
      double s = sqrt(1 - c * c); \
      BOOST_CHECK_SMALL(s, tolerance); \
    } \
  } \
}
