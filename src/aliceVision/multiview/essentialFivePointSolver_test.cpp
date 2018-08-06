// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <iostream>

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/multiview/projection.hpp"
#include "aliceVision/multiview/essentialFivePointSolver.hpp"
#include "aliceVision/multiview/NViewDataSet.hpp"

#define BOOST_TEST_MODULE essentialFivePointSolver
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

struct TestData {
  //-- Dataset that encapsulate :
  // 3D points and their projection given P1 and P2
  // Link between the two camera [R|t]
  Mat3X X;
  Mat3 R;
  Vec3 t;
  Mat3 E;
  Mat34 P1, P2;
  Mat2X x1, x2;
};

TestData SomeTestData() {
  TestData d;

  // --
  // Modeling random 3D points,
  // Consider first camera as [R=I|t=0],
  // Second camera as [R=Rx*Ry*Rz|t=random],
  // Compute projection of the 3D points onto image plane.
  // --
  d.X = Mat3X::Random(3,5);

  //-- Make point in front to the cameras.
  d.X.row(0).array() -= .5;
  d.X.row(1).array() -= .5;
  d.X.row(2).array() += 1.0;

  d.R = RotationAroundZ(0.3) * RotationAroundX(0.1) * RotationAroundY(0.2);
  d.t.setRandom();
  //d.t(1)  = 10;

  EssentialFromRt(Mat3::Identity(), Vec3::Zero(), d.R, d.t, &d.E);

  P_From_KRt(Mat3::Identity(), Mat3::Identity(), Vec3::Zero(), &d.P1);
  P_From_KRt(Mat3::Identity(), d.R, d.t, &d.P2);

  Project(d.P1, d.X, &d.x1);
  Project(d.P2, d.X, &d.x2);

  return d;
}

double EvalPolynomial(Vec p, double x, double y, double z) {
  return p(coef_xxx) * x * x * x
       + p(coef_xxy) * x * x * y
       + p(coef_xxz) * x * x * z
       + p(coef_xyy) * x * y * y
       + p(coef_xyz) * x * y * z
       + p(coef_xzz) * x * z * z
       + p(coef_yyy) * y * y * y
       + p(coef_yyz) * y * y * z
       + p(coef_yzz) * y * z * z
       + p(coef_zzz) * z * z * z
       + p(coef_xx)  * x * x
       + p(coef_xy)  * x * y
       + p(coef_xz)  * x * z
       + p(coef_yy)  * y * y
       + p(coef_yz)  * y * z
       + p(coef_zz)  * z * z
       + p(coef_x)   * x
       + p(coef_y)   * y
       + p(coef_z)   * z
       + p(coef_1)   * 1;
}

BOOST_AUTO_TEST_CASE(o1_Evaluation) {

  Vec p1 = Vec::Zero(20), p2 = Vec::Zero(20);
  p1(coef_x) = double(rand()) / RAND_MAX;
  p1(coef_y) = double(rand()) / RAND_MAX;
  p1(coef_z) = double(rand()) / RAND_MAX;
  p1(coef_1) = double(rand()) / RAND_MAX;
  p2(coef_x) = double(rand()) / RAND_MAX;
  p2(coef_y) = double(rand()) / RAND_MAX;
  p2(coef_z) = double(rand()) / RAND_MAX;
  p2(coef_1) = double(rand()) / RAND_MAX;

  Vec p3 = o1(p1, p2);

  for (double z = -5; z < 5; ++z) {
    for (double y = -5; y < 5; ++y) {
      for (double x = -5; x < 5; ++x) {
        BOOST_CHECK_SMALL(EvalPolynomial(p3, x, y, z)-
                    EvalPolynomial(p1, x, y, z) * EvalPolynomial(p2, x, y, z),
                    1e-8);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(o2_Evaluation) {

  Vec p1 = Vec::Zero(20), p2 = Vec::Zero(20);
  p1(coef_xx) = double(rand()) / RAND_MAX;
  p1(coef_xy) = double(rand()) / RAND_MAX;
  p1(coef_xz) = double(rand()) / RAND_MAX;
  p1(coef_yy) = double(rand()) / RAND_MAX;
  p1(coef_yz) = double(rand()) / RAND_MAX;
  p1(coef_zz) = double(rand()) / RAND_MAX;
  p1(coef_x)  = double(rand()) / RAND_MAX;
  p1(coef_y)  = double(rand()) / RAND_MAX;
  p1(coef_z)  = double(rand()) / RAND_MAX;
  p1(coef_1)  = double(rand()) / RAND_MAX;
  p2(coef_x)  = double(rand()) / RAND_MAX;
  p2(coef_y)  = double(rand()) / RAND_MAX;
  p2(coef_z)  = double(rand()) / RAND_MAX;
  p2(coef_1)  = double(rand()) / RAND_MAX;

  Vec p3 = o2(p1, p2);

  for (double z = -5; z < 5; ++z) {
    for (double y = -5; y < 5; ++y) {
      for (double x = -5; x < 5; ++x) {
        BOOST_CHECK_SMALL(EvalPolynomial(p3, x, y, z)-
                    EvalPolynomial(p1, x, y, z) * EvalPolynomial(p2, x, y, z),
                    1e-8);
      }
    }
  }
}

/// Check that the E matrix fit the Essential Matrix properties
/// Determinant is 0
///
#define EXPECT_ESSENTIAL_MATRIX_PROPERTIES(E, expectedPrecision) { \
  BOOST_CHECK_SMALL(E.determinant(), expectedPrecision); \
  Mat3 O = 2 * E * E.transpose() * E - (E * E.transpose()).trace() * E; \
  Mat3 zero3x3 = Mat3::Zero(); \
  EXPECT_MATRIX_NEAR(zero3x3, O, expectedPrecision);\
}

BOOST_AUTO_TEST_CASE(FivePointsRelativePose_Random) {

  TestData d = SomeTestData();

  std::vector<Mat3> Es;
  std::vector<Mat3> Rs;
  std::vector<Vec3> ts;
  FivePointsRelativePose(d.x1, d.x2, &Es);

  // Recover rotation and translation from E
  Rs.resize(Es.size());
  ts.resize(Es.size());
  for (size_t s = 0; s < Es.size(); ++s) {
    Vec2 x1Col = d.x1.col(0);
    Vec2 x2Col = d.x2.col(0);
    BOOST_CHECK(
      MotionFromEssentialAndCorrespondence(Es[s],
                                         Mat3::Identity(),
                                         x1Col,
                                         Mat3::Identity(),
                                         x2Col,
                                         &Rs[s],
                                         &ts[s]));
  }

  bool bsolution_found = false;
  for (size_t i = 0; i < Es.size(); ++i) {

    std::cout << i << std::endl;

    // Check that we find the correct relative orientation.
    if (FrobeniusDistance(d.R, Rs[i]) < 1e-3
        && (d.t / d.t.norm() - ts[i] / ts[i].norm()).norm() < 1e-3 ) {
      bsolution_found = true;
      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[i], 1e-8);
    }
  }
  //-- Almost one solution must find the correct relative orientation
  BOOST_CHECK(bsolution_found);
}

BOOST_AUTO_TEST_CASE(FivePointsRelativePose_test_data_sets) {

  //-- Setup a circular camera rig and assert that 5PT relative pose works.
  const int iNviews = 5;
  NViewDataSet d = NRealisticCamerasRing(iNviews, 5,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  // Compute pose [R|t] from 0 to [1;..;iNviews]
  for(int i=1; i <iNviews; ++i)
  {
    std::vector<Mat3> Es, Rs;  // Essential, Rotation matrix.
    std::vector<Vec3> ts;      // Translation matrix.
    FivePointsRelativePose(d._x[0], d._x[i], &Es);

    // Recover rotation and translation from E.
    Rs.resize(Es.size());
    ts.resize(Es.size());
    for (size_t s = 0; s < Es.size(); ++s) {
      Vec2 x1Col = d._x[0].col(0);
      Vec2 x2Col = d._x[i].col(0);
      BOOST_CHECK(
        MotionFromEssentialAndCorrespondence(Es[s],
        d._K[0],
        x1Col,
        d._K[i],
        x2Col,
        &Rs[s],
        &ts[s]));
    }
    //-- Compute Ground Truth motion
    Mat3 R;
    Vec3 t, t0 = Vec3::Zero(), t1 = Vec3::Zero();
    RelativeCameraMotion(d._R[0], d._t[0], d._R[i], d._t[i], &R, &t);

    // Assert that found relative motion is correct for almost one model.
    bool bsolution_found = false;
    for (size_t nModel = 0; nModel < Es.size(); ++nModel) {

      // Check that E holds the essential matrix constraints.
      EXPECT_ESSENTIAL_MATRIX_PROPERTIES(Es[nModel], 1e-8);

      // Check that we find the correct relative orientation.
      if (FrobeniusDistance(R, Rs[nModel]) < 1e-3
        && (t / t.norm() - ts[nModel] / ts[nModel].norm()).norm() < 1e-3 ) {
          bsolution_found = true;
      }
    }
    //-- Almost one solution must find the correct relative orientation
    BOOST_CHECK(bsolution_found);
  }
}

BOOST_AUTO_TEST_CASE(FivePointsNullspaceBasis_SatisfyEpipolarConstraint) {

  TestData d = SomeTestData();

  Mat E_basis = FivePointsNullspaceBasis(d.x1, d.x2);

  for (int s = 0; s < 4; ++s) {
    Mat3 E;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        E(i, j) = E_basis(3 * i + j, s);
      }
    }
    for (int i = 0; i < d.x1.cols(); ++i) {
      Vec3 x1(d.x1(0,i), d.x1(1,i), 1);
      Vec3 x2(d.x2(0,i), d.x2(1,i), 1);
      BOOST_CHECK_SMALL(x2.dot(E * x1), 1e-6);
    }
  }
}
