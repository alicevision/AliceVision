// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>
#include <aliceVision/system/Logger.hpp>
#include "aliceVision/numeric/numeric.hpp"

#define BOOST_TEST_MODULE numeric

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace std;

//-- Assert that stream interface is available
BOOST_AUTO_TEST_CASE ( TinyMatrix_print )
{
  Mat3 testMatrix = Mat3::Identity();
  ALICEVISION_LOG_DEBUG(testMatrix);
}

BOOST_AUTO_TEST_CASE ( TinyMatrix_checkIdentity )
{
  Mat3 testMatrix, expected;

  // build expected matrix and the test matrix
  expected.fill(0);
  expected(0,0) = expected(1,1) = expected(2,2) = 1.0;

  testMatrix.setIdentity();
  ALICEVISION_LOG_DEBUG(testMatrix);
  //-- Compare expected to the testMatrix.
  EXPECT_MATRIX_NEAR( expected, testMatrix, 1e-8);
}

BOOST_AUTO_TEST_CASE ( TinyMatrix_product )
{
  Mat3 a, b, expected;

  // build expected matrix and the test matrix
  a(0,0) = 1.0; a(0,1) = 2.0; a(0,2) = 3.0;
  a(1,0) = 4.0; a(1,1) = 5.0; a(1,2) = 6.0;
  a(2,0) = 7.0; a(2,1) = 8.0; a(2,2) = 9.0;

  b(0,0) = 10.0; b(0,1) = 11.0; b(0,2) = 12.0;
  b(1,0) = 13.0; b(1,1) = 14.0; b(1,2) = 15.0;
  b(2,0) = 16.0; b(2,1) = 17.0; b(2,2) = 18.0;

  Mat3 resAxB = a*b;
  Mat3 expected_resAxB;
  {
    Mat3 & t = expected_resAxB;
    t(0,0) = 84.0;  t(0,1) = 90.0;    t(0,2) = 96.0;
    t(1,0) = 201.0; t(1,1) = 216.0;  t(1,2) = 231.0;
    t(2,0) = 318.0; t(2,1) = 342.0;  t(2,2) = 366.0;
  }

  Mat3 resBxA = b*a;
  Mat3 expected_resBxA;
  {
    Mat3 & t = expected_resBxA;
    t(0,0) = 138; t(0,1) = 171;  t(0,2) = 204;
    t(1,0) = 174; t(1,1) = 216;  t(1,2) = 258;
    t(2,0) = 210; t(2,1) = 261;  t(2,2) = 312;
  }

  //-- Tests
  EXPECT_MATRIX_NEAR( expected_resAxB, resAxB, 1e-8);
  EXPECT_MATRIX_NEAR( expected_resBxA, resBxA, 1e-8);
}

BOOST_AUTO_TEST_CASE(TinyMatrix_LookAt) {
  // Simple orthogonality check.
  Vec3 e; e[0]= 1; e[1] = 2; e[2] = 3;
  Mat3 R = LookAt(e);
  Mat3 I = Mat3::Identity();
  Mat3 RRT = R*R.transpose();
  Mat3 RTR = R.transpose()*R;

  EXPECT_MATRIX_NEAR(I, RRT, 1e-15);
  EXPECT_MATRIX_NEAR(I, RTR, 1e-15);
}

BOOST_AUTO_TEST_CASE(Numeric_ExtractColumns) {
  Mat2X A(2, 5);
  A << 1, 2, 3, 4, 5,
       6, 7, 8, 9, 10;
  Vec2i columns; columns << 0, 2;
  Mat2X extracted = ExtractColumns(A, columns);
  BOOST_CHECK_SMALL(1- extracted(0,0), 1e-15);
  BOOST_CHECK_SMALL(3- extracted(0,1), 1e-15);
  BOOST_CHECK_SMALL(6- extracted(1,0), 1e-15);
  BOOST_CHECK_SMALL(8- extracted(1,1), 1e-15);
}

BOOST_AUTO_TEST_CASE(Numeric_MeanAndVarianceAlongRows) {
  int n = 4;
  Mat points(2,n);
  points << 0, 0, 1, 1,
    0, 2, 1, 3;

  Vec mean, variance;
  MeanAndVarianceAlongRows(points, &mean, &variance);

  BOOST_CHECK_SMALL(0.5-mean(0), 1e-8);
  BOOST_CHECK_SMALL(1.5-mean(1), 1e-8);
  BOOST_CHECK_SMALL(0.25-variance(0), 1e-8);
  BOOST_CHECK_SMALL(1.25-variance(1), 1e-8);
}
