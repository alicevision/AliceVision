// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/multiview/affineSolver.hpp>

#define BOOST_TEST_MODULE multiviewAffineSolver

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::multiview;

BOOST_AUTO_TEST_CASE(Affine2DTest_TranslationX) {
  Mat x1(2, 3);
  x1 <<  0, 1, 2,
         0, 1, 1;

  Mat x2(2, 3);
  x2 <<  1, 2, 3,
         0, 1, 1;

  Mat3 AffineMat;
  BOOST_CHECK(affine2DFromCorrespondencesLinear(x1, x2, &AffineMat));
  ALICEVISION_LOG_DEBUG("Mat Affine2D:\n" << AffineMat);
  Mat3 ground_truth;
  ground_truth << 1,0,1,
                  0,1,0,
                  0,0,1;
  EXPECT_MATRIX_NEAR(AffineMat, ground_truth,1e-8);
}

BOOST_AUTO_TEST_CASE(Affine2DTest_TranslationXY) {
  Mat x1(2, 3);
  x1 <<  0, 1, 2,
         0, 1, 1;

  Mat x2(2, 3);
  x2 <<  1, 2, 3,
         1, 2, 2;

  Mat3 affine_mat;
  BOOST_CHECK(affine2DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine2D:\n" << affine_mat);
  Mat3 ground_truth;
  ground_truth << 1,0,1,
                  0,1,1,
                  0,0,1;
  EXPECT_MATRIX_NEAR(affine_mat, ground_truth,1e-8);
}

BOOST_AUTO_TEST_CASE(Affine2DTest_Rotation45) {
  Mat x1(2, 4);
  x1 <<  0, 1, 2, 5,
         0, 1, 2, 3;

  double angle = 45.0;
  Mat3 rot;
  rot <<  cos(angle), -sin(angle), 0,
          sin(angle),  cos(angle), 0,
          0,           0,          1;
  Mat x2 = x1;
  for(int i = 0; i < x2.cols(); ++i)  {
    x2.block<2,1>(0,i) = rot.block<2,2>(0,0) * x1.col(i) ;
  }

  Mat3 affine_mat;
  BOOST_CHECK(affine2DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine2D " << affine_mat);
  EXPECT_MATRIX_NEAR(affine_mat, rot, 1e-8);
}

BOOST_AUTO_TEST_CASE(Affine2DTest_Rotation45AndTranslationXY) {
  Mat x1(2, 4);
  x1 <<  0, 1, 2, 5,
         0, 1, 2, 3;

  double angle = 45.0;
  Mat3 rot;
  rot <<  cos(angle),  sin(angle), -2,
         -sin(angle),  cos(angle), 5,
          0,           0,          1;

  Mat x2 = x1;
  // Transform point from ground truth matrix
  for(int i = 0; i < x2.cols(); ++i)  {
    x2.block<2,1>(0,i) = rot.block<2,2>(0,0) *  x1.col(i);// rot
    x2.block<2,1>(0,i) += rot.block<2,1>(0,2); // translation
  }

  Mat3 affine_mat;
  BOOST_CHECK(affine2DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine2D:\n" << affine_mat);
  EXPECT_MATRIX_NEAR(affine_mat, rot, 1e-8);
}

BOOST_AUTO_TEST_CASE(Affine2DTest_AffineGeneral) {
  Mat x1(2, 4);
  x1 <<  0, 1, 2, 5,
         0, 1, 2, 3;
  Mat3 m;
  m <<   3, -1,  4,
         6, -2, -3,
         0,  0,  1;

  Mat x2 = x1;
  // Transform point from ground truth matrix
  for(int i = 0; i < x2.cols(); ++i)  {
    x2.block<2,1>(0,i) = m.block<2,2>(0,0) * x1.col(i);// affine
    x2.block<2,1>(0,i) += m.block<2,1>(0,2); // translation
  }

  Mat3 affine_mat;
  BOOST_CHECK(affine2DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine2D:\n" << affine_mat);
  EXPECT_MATRIX_NEAR(affine_mat, m, 1e-8);
}

BOOST_AUTO_TEST_CASE(Affine3DTest_TranslationZ) {
  Mat x1(3, 4);
  x1 <<  0, 1, 2, 3,
         0, 5, 1, 3,
         0, 1, 7, 3;

  Mat x2(3, 4);
  x2 <<  0, 1, 2, 3,
         0, 5, 1, 3,
         1, 2, 8, 4;

  Mat4 AffineMat;
  BOOST_CHECK(affine3DFromCorrespondencesLinear(x1, x2, &AffineMat));
  ALICEVISION_LOG_DEBUG("Mat Affine3D:\n" << AffineMat);
  Mat4 ground_truth;
  ground_truth << 1,0,0,0,
                  0,1,0,0,
                  0,0,1,1,
                  0,0,0,1;
  EXPECT_MATRIX_NEAR(AffineMat, ground_truth,1e-8);
}

BOOST_AUTO_TEST_CASE(Affine3DTest_TranslationXYZ) {
  Mat x1(3, 4);
  x1 <<  0, 1, 2, 3,
         0, 5, 1, 3,
         0, 1, 7, 3;

  Mat x2(3, 4);
  x2 <<  2, 3, 4, 5,
        -1, 4, 0, 2,
         1, 2, 8, 4;

  Mat4 affine_mat;
  BOOST_CHECK(affine3DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine3D:\n" << affine_mat);
  Mat4 ground_truth;
  ground_truth << 1,0,0, 2,
                  0,1,0,-1,
                  0,0,1, 1,
                  0,0,0, 1;
  EXPECT_MATRIX_NEAR(affine_mat, ground_truth,1e-8);
}

BOOST_AUTO_TEST_CASE(Affine3DTest_RotationAndTranslationXYZ) {
  Mat x1(3, 4);
  x1 <<  0, 1, 2, 5,
         0, 1, 2, 3,
         0, 2, 0, 1;

  Mat4 M;
  M.setIdentity();
  /*
  M = AngleAxisd(45.0, Vector3f::UnitZ())
    * AngleAxisd(25.0, Vector3f::UnitX())
    * AngleAxisd(5.0, Vector3f::UnitZ());*/

  // Rotation on x + translation
  double angle = 45.0;
  Mat4 rot;
  rot <<  1,          0,           0, 1,
          0, cos(angle), -sin(angle), 3,
          0, sin(angle),  cos(angle),-2,
          0,          0,           0, 1;
  M *= rot;
  // Rotation on y
  angle = 25.0;
  rot <<  cos(angle), 0, sin(angle),  0,
          0,          1,          0,  0,
         -sin(angle), 0, cos(angle),  0,
          0,           0,          0, 1;
  M *= rot;
  // Rotation on z
  angle = 5.0;
  rot <<  cos(angle), -sin(angle), 0, 0,
          sin(angle),  cos(angle), 0, 0,
          0,           0,          1, 0,
          0,           0,          0, 1;
  M *= rot;
  Mat x2 = x1;
  // Transform point from ground affine matrix
  for(int i = 0; i < x2.cols(); ++i)  {
    x2.block<3,1>(0,i) =  M.block<3,3>(0,0) * x1.col(i) ;
    x2.block<3,1>(0,i) += M.block<3,1>(0,3); // translation
  }

  Mat4 affine_mat;
  BOOST_CHECK(affine3DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine3D:\n" << affine_mat);
  EXPECT_MATRIX_NEAR(affine_mat, M, 1e-8);
}

BOOST_AUTO_TEST_CASE(Affine3DTest_AffineGeneral) {
  Mat x1(3, 4);
  x1 <<  0, 1, 2, 5,
         0, 1, 2, 3,
         2, 0, 1, 2;
  Mat4 m;
  m <<   3, -1,  4, 1,
         6, -2, -3,-6,
         1,  0,  1, 2,
         0,  0,  0, 1;

  Mat x2 = x1;
  // Transform point from ground truth affine matrix
  for(int i = 0; i < x2.cols(); ++i)  {
    x2.block<3,1>(0,i) = m.block<3,3>(0,0) * x1.col(i);// affine
    x2.block<3,1>(0,i) += m.block<3,1>(0,3); // translation
  }

  Mat4 affine_mat;
  BOOST_CHECK(affine3DFromCorrespondencesLinear(x1, x2, &affine_mat));
  ALICEVISION_LOG_DEBUG("Mat Affine3D:\n" << affine_mat);
  EXPECT_MATRIX_NEAR(affine_mat, m, 1e-8);
}
