// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/multiview/NViewDataSet.hpp>

#include <aliceVision/multiview/resection/ResectionKernel.hpp>
#include <aliceVision/multiview/resection/EPnPKernel.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>

#define BOOST_TEST_MODULE ResectionKernel

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::multiview;

BOOST_AUTO_TEST_CASE(Resection_Kernel_Multiview)
{

  const int nViews = 3;
  const int nbPoints = 10;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  const int nResectionCameraIndex = 2;

  // Solve the problem and check that fitted value are good enough
  {
    Mat x = d._x[nResectionCameraIndex];
    Mat X = d._X;
    resection::Resection6PKernel kernel(x, X);

    std::size_t samples_[6]={0,1,2,3,4,5};
    std::vector<std::size_t> samples(samples_,samples_+6);
    std::vector<robustEstimation::Mat34Model> Ps;

    kernel.fit(samples, Ps);

    for (std::size_t i = 0; i < x.cols(); ++i)
    {
      BOOST_CHECK_SMALL(kernel.error(i, Ps.at(0)), 1e-8);
    }

    BOOST_CHECK_EQUAL(1, Ps.size());

    // Check that Projection matrix is near to the GT :
    Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array() / d.P(nResectionCameraIndex).norm();
    Mat34 COMPUTED_ProjectionMatrix = Ps.at(0).getMatrix().array() / Ps.at(0).getMatrix().norm();
    EXPECT_MATRIX_NEAR(GT_ProjectionMatrix, COMPUTED_ProjectionMatrix, 1e-8);
  }
}

/*
BOOST_AUTO_TEST_CASE(P3P_Kneip_CVPR11_Multiview)
{
  const int nViews = 3;
  const int nbPoints = 3;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  const int nResectionCameraIndex = 2;

  // Solve the problem and check that fitted value are good enough
  {
    Mat x = d._x[nResectionCameraIndex];
    Mat X = d._X;
    resection::P3PResectionKernel_K kernel(x, X, d._K[0]);

    std::size_t samples_[3]={0,1,2};
    std::vector<std::size_t> samples(samples_, samples_+3);
    std::vector<robustEstimation::Mat34Model> Ps;
    kernel.fit(samples, Ps);

    bool bFound = false;
    char index = -1;
    for (std::size_t i = 0; i < Ps.size(); ++i)
    {
      Mat34 GT_ProjectionMatrix = d.P(nResectionCameraIndex).array()
                                / d.P(nResectionCameraIndex).norm();
      Mat34 COMPUTED_ProjectionMatrix = Ps.at(i).getMatrix().array() / Ps.at(i).getMatrix().norm();
      if ( NormLInfinity(GT_ProjectionMatrix - COMPUTED_ProjectionMatrix) < 1e-8 )
      {
        bFound = true;
        index = i;
      }
    }
    BOOST_CHECK(bFound);

    // Check that for the found matrix residual is small
    for (std::size_t i = 0; i < x.cols(); ++i)
    {
      BOOST_CHECK_SMALL(kernel.error(i,Ps.at(index)), 1e-8);
    }
  }
}
*/

// Create a new synthetic dataset for the EPnP implementation.
// It seems it do not perform well on translation like t = (0,0,x)

// Generates all necessary inputs and expected outputs for EuclideanResection.
void CreateCameraSystem(const Mat3& KK,
                        const Mat3X& x_image,
                        const Vec& X_distances,
                        const Mat3& R_input,
                        const Vec3& T_input,
                        Mat2X *x_camera,
                        Mat3X *X_world,
                        Mat3  *R_expected,
                        Vec3  *T_expected)
{
  const auto num_points = x_image.cols();

  Mat3X x_unit_cam(3, num_points);
  x_unit_cam = KK.inverse() * x_image;

  // Create normalized camera coordinates to be used as an input to the PnP
  // function, instead of using NormalizeColumnVectors(&x_unit_cam).
  *x_camera = x_unit_cam.block(0, 0, 2, num_points);

  for (int i = 0; i < num_points; ++i)
  {
    x_unit_cam.col(i).normalize();
  }

  // Create the 3D points in the camera system.
  Mat X_camera(3, num_points);
  for (int i = 0; i < num_points; ++i)
  {
    X_camera.col(i) = X_distances(i) * x_unit_cam.col(i);
  }

  // Apply the transformation to the camera 3D points
  Mat translation_matrix(3, num_points);
  translation_matrix.row(0).setConstant(T_input(0));
  translation_matrix.row(1).setConstant(T_input(1));
  translation_matrix.row(2).setConstant(T_input(2));

  *X_world = R_input * X_camera + translation_matrix;

  // Create the expected result for comparison.
  *R_expected = R_input.transpose();
  *T_expected = *R_expected * ( - T_input);
}


BOOST_AUTO_TEST_CASE(EuclideanResection_Points6AllRandomInput) {
  Mat3 KK;
  KK << 2796, 0,    800,
        0 ,   2796, 600,
        0,    0,    1;

  // Create random image points for a 1600x1200 image.
  int w = 1600;
  int h = 1200;
  int num_points = 6;
  Mat3X x_image(3, num_points);
  x_image.row(0) = w * Vec::Random(num_points).array().abs();
  x_image.row(1) = h * Vec::Random(num_points).array().abs();
  x_image.row(2).setOnes();

  // Normalized camera coordinates to be used as an input to the PnP function.
  Mat2X x_camera;
  Vec X_distances = 100 * Vec::Random(num_points).array().abs();

  // Create the random camera motion R and t that resection should recover.
  Mat3 R_input;
  R_input = Eigen::AngleAxisd(rand(), Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(rand(), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(rand(), Eigen::Vector3d::UnitZ());

  Vec3 T_input;
  T_input = T_input.setRandom().array() * 100;

  // Create the camera system.
  Mat3 R_expected;
  Vec3 T_expected;
  Mat3X X_world;
  CreateCameraSystem(KK, x_image, X_distances, R_input, T_input,
                     &x_camera, &X_world, &R_expected, &T_expected);


  {
    using Kernel = resection::EPnPKernel;
    Kernel kernel(x_image.block(0, 0, 2, 6), X_world, KK);

    std::size_t samples_[6]={0,1,2,3,4,5};
    std::vector<std::size_t> samples(samples_, samples_+6);
    std::vector<robustEstimation::Mat34Model> Ps;
    kernel.fit(samples, Ps);

    BOOST_CHECK_EQUAL(1, Ps.size());

    bool bFound = false;
    for (std::size_t i = 0; i < Ps.size(); ++i)
    {
      Mat3 R_output;
      Vec3 T_output;
      Mat3 K;
      KRt_from_P(Ps.at(i).getMatrix(), &K, &R_output, &T_output);
      if(NormLInfinity(T_output-T_expected) < 1e-8 && NormLInfinity(R_output-R_expected) < 1e-8)
        bFound = true;
    }
    BOOST_CHECK(bFound);
  }

}
