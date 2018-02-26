// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/multiview/NViewDataSet.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/multiview/knownRotationTranslationKernel.hpp"

#include <vector>

#define BOOST_TEST_MODULE knownRotationTranslationKernel
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

// Estimate the translation for a pair of view for which the relative rotation is known
// Use a 2 correspondences based solver
BOOST_AUTO_TEST_CASE(Translation_knownRotation_Kernel_Multiview) {

  const int nViews = 10;
  const int nbPoints = 2;
  const NViewDataSet d = NRealisticCamerasRing(nViews, nbPoints,
    NViewDatasetConfigurator(1,1,0,0,5,0)); // Suppose a camera with Unit matrix as K

  // Solve the problem and check that fitted value are good enough
  for (int nCameraIndex = 2; nCameraIndex < nViews; ++nCameraIndex)
  {
    const Mat x0 = d._x[0];
    const Mat xCam = d._x[nCameraIndex];
    // coordinates does not need to be normalized since we have used a unit K matrix.

    // Compute GT (Ground Truth) motion
    Mat3 R_GT;
    Vec3 t_GT;
    RelativeCameraMotion(d._R[0], d._t[0], d._R[nCameraIndex], d._t[nCameraIndex], &R_GT, &t_GT);

    aliceVision::translation::kernel::TranslationFromKnowRotationKernel kernel(x0, xCam, R_GT);

    std::size_t samples_[2]={0,1};
    std::vector<std::size_t> samples(samples_,samples_+2);
    std::vector<Vec3> vec_t;
    kernel.Fit(samples, &vec_t);

    BOOST_CHECK_EQUAL(1, vec_t.size());

    // Check that the fitted model is compatible with the data
    // Here the distance to the epipolar line is used
    for (std::size_t i = 0; i < x0.cols(); ++i) {
      BOOST_CHECK_SMALL(kernel.Error(i, vec_t[0]), 1e-8);
    }

    // Check that the GT translation and the estimated one are equal
    EXPECT_MATRIX_NEAR(t_GT.normalized(), vec_t[0].normalized(), 1e-8);
  }
}
